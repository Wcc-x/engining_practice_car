/**
 * @file    ax_ble.c
 * @brief   BLE 蓝牙接收 — NimBLE GATT Server (NUS) + 江协科技数据包解析
 *
 * 架构:
 *   NimBLE Host (GATT Server)
 *     └── Nordic UART Service (NUS)
 *           ├── RX Characteristic (Write, phone→ESP)  ← 小程序写入数据包
 *           └── TX Characteristic (Notify, ESP→phone)  → 调试回传
 *
 * 数据流:
 *   小程序发送 "[joystick,100,200,-50,0]\r\n"
 *     → NUS RX Write callback
 *       → 字节累积到环形缓冲区
 *         → 检测完整数据包 ([...]+换行)
 *           → 解析逗号分隔字段
 *             → 更新 ax_ble_joystick / ax_ble_slider / ax_ble_key
 *               → main 循环中读取并控制小车
 */

#include "ax_ble.h"
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"

/* NimBLE */
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "services/gatt/ble_svc_gatt.h"

static const char *TAG = "AX_BLE";

/* ============================================================
 *  NUS UUID (Nordic UART Service — 标准)
 * ============================================================ */
#define NUS_SERVICE_UUID        0x6E400001B5A3F393E0A9E50E24DCCA9E
#define NUS_RX_CHAR_UUID        0x6E400002B5A3F393E0A9E50E24DCCA9E  /* phone→ESP, Write */
#define NUS_TX_CHAR_UUID        0x6E400003B5A3F393E0A9E50E24DCCA9E  /* ESP→phone, Notify */

/* NimBLE 属性句柄 */
static uint16_t nus_rx_handle;
static uint16_t nus_tx_handle;
static uint16_t nus_cccd_handle;

/* 连接句柄 (0xFFFF = 未连接) */
static uint16_t conn_handle = BLE_HS_CONN_HANDLE_NONE;

/* ============================================================
 *  全局数据
 * ============================================================ */
AX_BLE_Joystick  ax_ble_joystick;
AX_BLE_Slider    ax_ble_slider;
AX_BLE_Key       ax_ble_key;
bool             ax_ble_connected = false;

/* 接收缓冲区 (用于调试查看原始数据) */
#define RX_BUF_SIZE  512
uint8_t  ax_ble_rx_buf[RX_BUF_SIZE];
uint16_t ax_ble_rx_len = 0;

/* 数据包累积缓冲区 */
static char   pkt_buf[256];
static uint8_t pkt_idx = 0;

/* ============================================================
 *  内部函数声明
 * ============================================================ */
static int  nus_gap_event_cb(struct ble_gap_event *event, void *arg);
static int  nus_gatt_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                                struct ble_gatt_access_ctxt *ctxt, void *arg);
static void ax_ble_advertise(void);
static void ax_ble_parse_packet(const char *str, uint8_t len);

/* ============================================================
 *  GATT 服务定义 (NUS)
 * ============================================================ */

/* NUS Service 定义 */
static const struct ble_gatt_svc_def nus_gatt_svcs[] = {
    {
        /* NUS Service */
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID128_DECLARE(NUS_SERVICE_UUID),
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                /* RX Characteristic — 手机写入数据到 ESP32 */
                .uuid = BLE_UUID128_DECLARE(NUS_RX_CHAR_UUID),
                .access_cb = nus_gatt_access_cb,
                .flags = BLE_GATT_CHR_F_WRITE,
                .val_handle = &nus_rx_handle,
            },
            {
                /* TX Characteristic — ESP32 通知数据到手机 */
                .uuid = BLE_UUID128_DECLARE(NUS_TX_CHAR_UUID),
                .access_cb = nus_gatt_access_cb,
                .flags = BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &nus_tx_handle,
                .descriptors = (struct ble_gatt_dsc_def[]) {
                    {
                        .uuid = BLE_UUID16_DECLARE(BLE_GATT_DSC_CLT_CFG_UUID16),
                        .att_flags = BLE_ATT_F_READ | BLE_ATT_F_WRITE,
                        .access_cb = nus_gatt_access_cb,
                        .handle = &nus_cccd_handle,
                    },
                    { 0 } /* terminator */
                },
            },
            { 0 } /* terminator */
        },
    },
    { 0 } /* terminator */
};

/* ============================================================
 *  GATT 访问回调 — 处理 Write / Read / Notify CCCD
 * ============================================================ */
static int nus_gatt_access_cb(uint16_t c_handle, uint16_t attr_handle,
                               struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (attr_handle == nus_rx_handle) {
        /* 手机→ESP32: 写入数据 */
        if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
            uint16_t om_len = OS_MBUF_PKTLEN(ctxt->om);
            if (om_len > 0 && om_len < RX_BUF_SIZE) {
                /* 拷贝到调试缓冲区 */
                ax_ble_rx_len = om_len;
                os_mbuf_copyd(ctxt->om, 0, om_len, ax_ble_rx_buf);
                ax_ble_rx_buf[om_len] = '\0';

                /* 逐字节累积到数据包缓冲区 */
                for (uint16_t i = 0; i < om_len; i++) {
                    char ch = (char)ax_ble_rx_buf[i];

                    /* 检测包头 '[' — 开始新数据包 */
                    if (ch == '[') {
                        pkt_idx = 0;
                        pkt_buf[pkt_idx++] = ch;
                    }
                    /* 包内数据 — 累积 */
                    else if (pkt_idx > 0 && pkt_idx < sizeof(pkt_buf) - 1) {
                        pkt_buf[pkt_idx++] = ch;

                        /* 检测包尾 ']' — 完整数据包接收完成 */
                        if (ch == ']') {
                            pkt_buf[pkt_idx] = '\0';
                            ax_ble_parse_packet(pkt_buf + 1, pkt_idx - 2);  /* 跳过[和] */
                            pkt_idx = 0;
                        }
                    }
                    /* 未在包内的字节 — 忽略 */
                }
            }
            return 0;
        }
    }

    /* 允许 CCCD 读写 (TX Notify 使能) */
    if (attr_handle == nus_cccd_handle) {
        return 0;
    }

    return BLE_ATT_ERR_UNLIKELY;
}

/* ============================================================
 *  GAP 事件回调 — 连接/断开
 * ============================================================ */
static int nus_gap_event_cb(struct ble_gap_event *event, void *arg)
{
    (void)arg;

    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        /* 连接成功 */
        if (event->connect.status == 0) {
            ESP_LOGI(TAG, "Connected!");
            conn_handle = event->connect.conn_handle;
            ax_ble_connected = true;
        } else {
            /* 连接失败 — 重新开始广播 */
            ax_ble_advertise();
        }
        break;

    case BLE_GAP_EVENT_DISCONNECT:
        /* 断开连接 */
        ESP_LOGI(TAG, "Disconnected, re-advertising...");
        conn_handle = BLE_HS_CONN_HANDLE_NONE;
        ax_ble_connected = false;
        ax_ble_advertise();
        break;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        /* 广播超时 → 重新开始 */
        ax_ble_advertise();
        break;

    case BLE_GAP_EVENT_SUBSCRIBE:
        /* 手机订阅 Notify */
        if (event->subscribe.attr_handle == nus_cccd_handle) {
            ESP_LOGI(TAG, "Client subscribed to TX notify");
        }
        break;

    default:
        break;
    }

    return 0;
}

/* ============================================================
 *  广播 — 可被发现和连接
 * ============================================================ */
static void ax_ble_advertise(void)
{
    struct ble_gap_adv_params adv_params = {0};
    struct ble_hs_adv_fields   fields     = {0};
    int rc;

    /* 广播数据: 设备名称 + 完整 UUID128 列表 */
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.name = (uint8_t *)"XTARK-ESP32";
    fields.name_len = strlen("XTARK-ESP32");
    fields.name_is_complete = 1;

    rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "adv_set_fields failed: %d", rc);
        return;
    }

    adv_params.conn_mode    = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode    = BLE_GAP_DISC_MODE_GEN;
    adv_params.itvl_min     = BLE_GAP_ADV_ITVL_MS(30);
    adv_params.itvl_max     = BLE_GAP_ADV_ITVL_MS(60);

    rc = ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER,
                           &adv_params, nus_gap_event_cb, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "adv_start failed: %d", rc);
    }
}

/* ============================================================
 *  Host 任务回调 — NimBLE 栈就绪时调用
 * ============================================================ */
static void nus_on_sync(void)
{
    /* 注册 GATT 服务 */
    int rc = ble_gatts_count_cfg(nus_gatt_svcs);
    if (rc != 0) {
        ESP_LOGE(TAG, "count_cfg failed: %d", rc);
        return;
    }
    rc = ble_gatts_add_svcs(nus_gatt_svcs);
    if (rc != 0) {
        ESP_LOGE(TAG, "add_svcs failed: %d", rc);
        return;
    }

    /* 开始广播 */
    ax_ble_advertise();
    ESP_LOGI(TAG, "BLE NUS Server started, advertising as 'XTARK-ESP32'");
}

/* ============================================================
 *  NimBLE Host 任务
 * ============================================================ */
static void nus_host_task(void *param)
{
    (void)param;
    ESP_LOGI(TAG, "NimBLE host task started");
    /* 此函数不会返回 — 内部运行 NimBLE 事件循环 */
    nimble_port_run();
    /* unreachable */
    vTaskDelete(NULL);
}

/* ============================================================
 *  公共 API: 初始化 BLE
 * ============================================================ */
void AX_BLE_Init(const char *device_name)
{
    (void)device_name;  /* 如需动态名称，可存入全局变量在 advertise 中使用 */

    /* 初始化 NVS (NimBLE 需要) */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    /* 初始化 NimBLE */
    nimble_port_init();

    /* 设置同步回调 (栈就绪后注册 GATT, 开始广播) */
    ble_hs_cfg.sync_cb = nus_on_sync;

    /* 启动 NimBLE Host 任务 (固定到 CPU 核 0, 优先级 5) */
    xTaskCreatePinnedToCore(nus_host_task, "nimble_host",
                            4096, NULL, 5, NULL, 0);

    /* 初始化全局数据 */
    memset(&ax_ble_joystick, 0, sizeof(ax_ble_joystick));
    memset(&ax_ble_slider,   0, sizeof(ax_ble_slider));
    memset(&ax_ble_key,      0, sizeof(ax_ble_key));
    pkt_idx = 0;
}

/* ============================================================
 *  公共 API: 发送数据到手机 (Notify)
 * ============================================================ */
int AX_BLE_Send(const uint8_t *data, uint16_t len)
{
    if (conn_handle == BLE_HS_CONN_HANDLE_NONE) {
        return -1;
    }

    struct os_mbuf *om = ble_hs_mbuf_from_flat(data, len);
    if (om == NULL) {
        return -2;
    }

    int rc = ble_gattc_notify_custom(conn_handle, nus_tx_handle, om);
    return rc;
}

/* ============================================================
 *  数据包解析器 — 解析江协科技小程序数据包
 * ============================================================
 *
 *  输入: "joystick,100,200,-50,0"  (已去掉 [ 和 ])
 *        或 "j,100,200,-50,0"       (短格式)
 *
 *  命令标识:
 *    joystick / j  → 摇杆控制
 *    slider   / s  → 滑杆控制
 *    key      / k  → 按键控制
 *    display  / d  → 显示屏  (暂忽略)
 *    plot     / p  → 绘图    (暂忽略)
 *    display-clear / d-c → 清屏 (暂忽略)
 *    plot-clear     / p-c → 清绘图 (暂忽略)
 */

/* 最大字段数 */
#define MAX_TOKENS  10

/* 从逗号分隔字符串中提取字段 */
static uint8_t tokenize(const char *str, uint8_t len, const char *tokens[], uint8_t max_tok)
{
    uint8_t count = 0;
    uint8_t start = 0;

    for (uint8_t i = 0; i <= len && count < max_tok; i++) {
        if (i == len || str[i] == ',') {
            if (i > start) {
                /* 跳过前导空格 */
                while (start < i && str[start] == ' ') start++;
                /* 跳过后缀空格 */
                uint8_t end = i;
                while (end > start && str[end - 1] == ' ') end--;

                if (end > start) {
                    /* 使用静态缓冲区存放字段副本 */
                    static char tok_buf[MAX_TOKENS][32];
                    uint8_t copy_len = (end - start) < 31 ? (end - start) : 31;
                    memcpy(tok_buf[count], str + start, copy_len);
                    tok_buf[count][copy_len] = '\0';
                    tokens[count] = tok_buf[count];
                    count++;
                }
            }
            start = i + 1;
        }
    }
    return count;
}

/* 判断字符串相等 (不区分大小写) */
static bool streq_nocase(const char *a, const char *b)
{
    while (*a && *b) {
        char ca = *a, cb = *b;
        if (ca >= 'A' && ca <= 'Z') ca += 32;
        if (cb >= 'A' && cb <= 'Z') cb += 32;
        if (ca != cb) return false;
        a++; b++;
    }
    return (*a == '\0' && *b == '\0');
}

static void ax_ble_parse_packet(const char *str, uint8_t len)
{
    if (len == 0 || str == NULL) return;

    /* 跳过前导空白 */
    while (len > 0 && *str == ' ') { str++; len--; }
    if (len == 0) return;

    /* 分割字段 */
    const char *tokens[MAX_TOKENS];
    uint8_t n = tokenize(str, len, tokens, MAX_TOKENS);
    if (n == 0) return;

    const char *cmd = tokens[0];

    /* ---- 摇杆: [joystick,lx,ly,rx,ry] 或 [j,lx,ly,rx,ry] ---- */
    if (streq_nocase(cmd, "joystick") || streq_nocase(cmd, "j")) {
        if (n >= 5) {
            ax_ble_joystick.left_x  = (int16_t)atoi(tokens[1]);
            ax_ble_joystick.left_y  = (int16_t)atoi(tokens[2]);
            ax_ble_joystick.right_x = (int16_t)atoi(tokens[3]);
            ax_ble_joystick.right_y = (int16_t)atoi(tokens[4]);
            ax_ble_joystick.valid   = true;
            ESP_LOGI(TAG, "Joystick: LX=%d LY=%d RX=%d RY=%d",
                     ax_ble_joystick.left_x, ax_ble_joystick.left_y,
                     ax_ble_joystick.right_x, ax_ble_joystick.right_y);
        }
        return;
    }

    /* ---- 滑杆: [slider,name,value] 或 [s,name,value] ---- */
    if (streq_nocase(cmd, "slider") || streq_nocase(cmd, "s")) {
        if (n >= 3) {
            ax_ble_slider.name  = (int16_t)atoi(tokens[1]);
            ax_ble_slider.value = (int16_t)atoi(tokens[2]);
            ax_ble_slider.valid = true;
            ESP_LOGI(TAG, "Slider: name=%d value=%d",
                     ax_ble_slider.name, ax_ble_slider.value);
        }
        return;
    }

    /* ---- 按键: [key,name,down/up] 或 [k,name,d/u] ---- */
    if (streq_nocase(cmd, "key") || streq_nocase(cmd, "k")) {
        if (n >= 3) {
            ax_ble_key.name = (int16_t)atoi(tokens[1]);
            /* "down"/"d" = 按下, "up"/"u" = 松开 */
            if (streq_nocase(tokens[2], "down") || streq_nocase(tokens[2], "d")) {
                ax_ble_key.is_down = true;
            } else {
                ax_ble_key.is_down = false;
            }
            ax_ble_key.valid = true;
            ESP_LOGI(TAG, "Key: name=%d %s",
                     ax_ble_key.name,
                     ax_ble_key.is_down ? "DOWN" : "UP");
        }
        return;
    }

    /* ---- display / plot — 暂不处理 ---- */
    ESP_LOGD(TAG, "Unhandled packet: [%.*s]", len, str);
}
