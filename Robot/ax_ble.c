/**
 * @file    ax_ble.c
 * @brief   BLE 蓝牙接收 — NimBLE GATT Server (NUS) + 江协科技数据包解析
 *
 * 基于 ESP-IDF v6.0.1 NimBLE 实现 Nordic UART Service (NUS) 外设角色。
 * 小程序作为 GATT Client 连接后，通过 NUS RX Characteristic 写入数据包，
 * ESP32 解析数据包并提取控制指令（摇杆/滑杆/按键）。
 *
 * 数据包格式（江协科技小程序规定）:
 *   [joystick,lx,ly,rx,ry]  或 [j,lx,ly,rx,ry]      摇杆
 *   [slider,name,value]      或 [s,name,value]        滑杆
 *   [key,name,down/up]       或 [k,name,d/u]          按键
 *
 * NUS UUID (Nordic UART Service 标准):
 *   Service UUID:      6E400001-B5A3-F393-E0A9-E50E24DCCA9E
 *   RX (phone→ESP):     6E400002-B5A3-F393-E0A9-E50E24DCCA9E  (Write)
 *   TX (ESP→phone):     6E400003-B5A3-F393-E0A9-E50E24DCCA9E  (Notify)
 */

#include "ax_ble.h"

#include <assert.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "esp_log.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* NimBLE */
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "host/util/util.h"
#include "nimble/ble.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

static const char *TAG = "AX_BLE";

/* ============================================================
 *  设备名称
 * ============================================================ */
#define DEVICE_NAME  "XTARK-ESP32"

/* ============================================================
 *  NUS UUID 128-bit (Nordic UART Service)
 *
 *  NimBLE 使用小端序存储 128-bit UUID:
 *  BLE_UUID128_INIT( byte[0], byte[1], ..., byte[15] )
 *  其中 byte[0] 是最低有效字节
 *
 *  标准格式: 6E400001-B5A3-F393-E0A9-E50E24DCCA9E
 *  MSB→LSB:  6E 40 00 01 B5 A3 F3 93 E0 A9 E5 0E 24 DC CA 9E
 *  LSB→MSB:  9E CA DC 24 0E E5 A9 E0 93 F3 A3 B5 01 00 40 6E
 * ============================================================ */

/* NUS Service UUID */
static const ble_uuid128_t nus_svc_uuid = BLE_UUID128_INIT(
    0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
    0x93, 0xF3, 0xA3, 0xB5, 0x01, 0x00, 0x40, 0x6E
);

/* RX Characteristic UUID (phone→ESP, Write) */
static const ble_uuid128_t nus_rx_uuid = BLE_UUID128_INIT(
    0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
    0x93, 0xF3, 0xA3, 0xB5, 0x02, 0x00, 0x40, 0x6E
);

/* TX Characteristic UUID (ESP→phone, Notify) */
static const ble_uuid128_t nus_tx_uuid = BLE_UUID128_INIT(
    0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
    0x93, 0xF3, 0xA3, 0xB5, 0x03, 0x00, 0x40, 0x6E
);

/* CCCD UUID (16-bit standard) */
static const ble_uuid16_t cccd_uuid = BLE_UUID16_INIT(BLE_GATT_DSC_CLT_CFG_UUID16);

/* ============================================================
 *  NimBLE GATT 句柄
 * ============================================================ */
static uint16_t nus_rx_val_handle;
static uint16_t nus_tx_val_handle;
static uint16_t nus_tx_cccd_handle;

/* 当前连接句柄 */
static uint16_t conn_handle = BLE_HS_CONN_HANDLE_NONE;
/* TX Notify 是否已启用 */
static bool tx_notify_enabled = false;

/* ============================================================
 *  全局数据 (对外)
 * ============================================================ */
AX_BLE_Joystick  ax_ble_joystick;
AX_BLE_Slider    ax_ble_slider;
AX_BLE_Key       ax_ble_key;
bool             ax_ble_connected = false;

#define RX_BUF_SIZE  512
uint8_t  ax_ble_rx_buf[RX_BUF_SIZE];
uint16_t ax_ble_rx_len = 0;

/* 数据包累积缓冲区 */
static char   pkt_buf[256];
static uint8_t pkt_idx = 0;

/* ============================================================
 *  前向声明
 * ============================================================ */
static int  nus_gap_event_handler(struct ble_gap_event *event, void *arg);
static int  nus_gatt_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                                struct ble_gatt_access_ctxt *ctxt, void *arg);
static void nus_start_advertising(void);
static void ax_ble_parse_packet(const char *str, uint8_t len);
static void nus_on_stack_sync(void);
static void nus_on_stack_reset(int reason);
void nus_gatt_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg);

/* ============================================================
 *  GATT 服务表 (NUS)
 * ============================================================ */
static const struct ble_gatt_svc_def nus_gatt_svcs[] = {
    {
        /* NUS Primary Service */
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &nus_svc_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                /* RX Characteristic (phone→ESP, Write) */
                .uuid = &nus_rx_uuid.u,
                .access_cb = nus_gatt_access_cb,
                .flags = BLE_GATT_CHR_F_WRITE,
                .val_handle = &nus_rx_val_handle,
            },
            {
                /* TX Characteristic (ESP→phone, Notify) */
                .uuid = &nus_tx_uuid.u,
                .access_cb = nus_gatt_access_cb,
                .flags = BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &nus_tx_val_handle,
                .descriptors = (struct ble_gatt_dsc_def[]) {
                    {
                        /* CCCD — 手机写入以启用 Notify */
                        .uuid = &cccd_uuid.u,
                        .att_flags = BLE_ATT_F_READ | BLE_ATT_F_WRITE,
                        .access_cb = nus_gatt_access_cb,
                        .handle = &nus_tx_cccd_handle,
                    },
                    { 0 } /* descriptors terminator */
                },
            },
            { 0 } /* characteristics terminator */
        },
    },
    { 0 } /* services terminator */
};

/* ============================================================
 *  GATT 注册回调
 * ============================================================ */
void nus_gatt_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg)
{
    (void)arg;
    switch (ctxt->op) {
    case BLE_GATT_REGISTER_OP_SVC:
        ESP_LOGD(TAG, "registered NUS service handle=%d", ctxt->svc.handle);
        break;
    case BLE_GATT_REGISTER_OP_CHR:
        ESP_LOGD(TAG, "registered characteristic def_handle=%d val_handle=%d",
                 ctxt->chr.def_handle, ctxt->chr.val_handle);
        break;
    case BLE_GATT_REGISTER_OP_DSC:
        ESP_LOGD(TAG, "registered descriptor handle=%d", ctxt->dsc.handle);
        break;
    default:
        break;
    }
}

/* ============================================================
 *  GATT 访问回调 — 处理 Write (RX) & CCCD
 * ============================================================ */
static int nus_gatt_access_cb(uint16_t c_handle, uint16_t attr_handle,
                               struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)arg;
    (void)c_handle;

    switch (ctxt->op) {

    /* ---- RX: 手机写入数据到 ESP32 ---- */
    case BLE_GATT_ACCESS_OP_WRITE_CHR:
        if (attr_handle == nus_rx_val_handle) {
            uint16_t om_len = OS_MBUF_PKTLEN(ctxt->om);
            if (om_len > 0 && om_len < RX_BUF_SIZE) {
                /* 拷贝到调试缓冲区 */
                os_mbuf_copyd(ctxt->om, 0, om_len, ax_ble_rx_buf);
                ax_ble_rx_len = om_len;
                ax_ble_rx_buf[om_len] = '\0';

                /* 逐字节累积到数据包缓冲区 */
                for (uint16_t i = 0; i < om_len; i++) {
                    char ch = (char)ax_ble_rx_buf[i];

                    if (ch == '[') {
                        /* 包头 — 开始新数据包 */
                        pkt_idx = 0;
                        pkt_buf[pkt_idx++] = ch;
                    } else if (pkt_idx > 0 && pkt_idx < sizeof(pkt_buf) - 1) {
                        pkt_buf[pkt_idx++] = ch;

                        /* 包尾 — 完整数据包 */
                        if (ch == ']') {
                            pkt_buf[pkt_idx] = '\0';
                            ax_ble_parse_packet(pkt_buf + 1, pkt_idx - 2);
                            pkt_idx = 0;
                        }
                    }
                }

                /* 回传确认到手机 (调试用) */
                if (tx_notify_enabled) {
                    struct os_mbuf *om = ble_hs_mbuf_from_flat(
                        ax_ble_rx_buf, om_len);
                    if (om) {
                        ble_gattc_notify_custom(conn_handle, nus_tx_val_handle, om);
                    }
                }
            }
            return 0;
        }
        break;

    /* ---- CCCD: 手机订阅/取消 Notify ---- */
    case BLE_GATT_ACCESS_OP_WRITE_DSC:
        if (attr_handle == nus_tx_cccd_handle) {
            if (ctxt->om->om_len >= 2) {
                uint16_t val = (ctxt->om->om_data[1] << 8) | ctxt->om->om_data[0];
                tx_notify_enabled = (val & 0x0001) != 0;
                ESP_LOGI(TAG, "TX notify %s", tx_notify_enabled ? "ENABLED" : "DISABLED");
            }
            return 0;
        }
        break;

    default:
        break;
    }

    return BLE_ATT_ERR_UNLIKELY;
}

/* ============================================================
 *  GAP 事件处理
 * ============================================================ */
static int nus_gap_event_handler(struct ble_gap_event *event, void *arg)
{
    (void)arg;

    switch (event->type) {

    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            ESP_LOGI(TAG, "BLE Connected!");
            conn_handle = event->connect.conn_handle;
            ax_ble_connected = true;
        } else {
            ESP_LOGI(TAG, "Connection failed, re-advertising...");
            nus_start_advertising();
        }
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "Disconnected (reason=%d), re-advertising...",
                 event->disconnect.reason);
        conn_handle = BLE_HS_CONN_HANDLE_NONE;
        ax_ble_connected = false;
        tx_notify_enabled = false;
        pkt_idx = 0;
        nus_start_advertising();
        return 0;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        ESP_LOGI(TAG, "Advertising complete, restarting...");
        nus_start_advertising();
        return 0;

    case BLE_GAP_EVENT_SUBSCRIBE:
        ESP_LOGI(TAG, "Subscribe event: conn=%d attr=%d notify=%d",
                 event->subscribe.conn_handle, event->subscribe.attr_handle,
                 event->subscribe.cur_notify);
        return 0;

    case BLE_GAP_EVENT_NOTIFY_TX:
        if (event->notify_tx.status != 0 &&
            event->notify_tx.status != BLE_HS_EDONE) {
            ESP_LOGW(TAG, "Notify TX error: status=%d", event->notify_tx.status);
        }
        return 0;

    case BLE_GAP_EVENT_MTU:
        ESP_LOGI(TAG, "MTU update: mtu=%d", event->mtu.value);
        return 0;

    default:
        return 0;
    }
}

/* ============================================================
 *  开始广播
 * ============================================================ */
static void nus_start_advertising(void)
{
    int rc;
    struct ble_hs_adv_fields adv_fields = {0};
    struct ble_gap_adv_params adv_params = {0};

    /* 广播数据 */
    adv_fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    adv_fields.name = (uint8_t *)DEVICE_NAME;
    adv_fields.name_len = strlen(DEVICE_NAME);
    adv_fields.name_is_complete = 1;

    rc = ble_gap_adv_set_fields(&adv_fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "failed to set adv fields: %d", rc);
        return;
    }

    /* 广播参数 */
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    adv_params.itvl_min  = BLE_GAP_ADV_ITVL_MS(30);
    adv_params.itvl_max  = BLE_GAP_ADV_ITVL_MS(60);

    rc = ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER,
                           &adv_params, nus_gap_event_handler, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "failed to start advertising: %d", rc);
        return;
    }

    ESP_LOGI(TAG, "Advertising started as '%s'", DEVICE_NAME);
}

/* ============================================================
 *  NimBLE 栈回调
 * ============================================================ */

/* 栈就绪 — 注册 GATT 服务 + 开始广播 */
static void nus_on_stack_sync(void)
{
    int rc;

    /* 确保有蓝牙地址 */
    rc = ble_hs_util_ensure_addr(0);
    if (rc != 0) {
        ESP_LOGE(TAG, "no BT address available");
        return;
    }

    /* 注册 GATT 服务 */
    rc = ble_gatts_count_cfg(nus_gatt_svcs);
    if (rc != 0) {
        ESP_LOGE(TAG, "gatts_count_cfg failed: %d", rc);
        return;
    }
    rc = ble_gatts_add_svcs(nus_gatt_svcs);
    if (rc != 0) {
        ESP_LOGE(TAG, "gatts_add_svcs failed: %d", rc);
        return;
    }

    /* 开始广播 */
    nus_start_advertising();
}

/* 栈复位 */
static void nus_on_stack_reset(int reason)
{
    ESP_LOGI(TAG, "NimBLE stack reset, reason=%d", reason);
}

/* ============================================================
 *  配置并启动 NimBLE Host
 * ============================================================ */
static void nimble_host_config_init(void)
{
    /* 回调 */
    ble_hs_cfg.reset_cb = nus_on_stack_reset;
    ble_hs_cfg.sync_cb  = nus_on_stack_sync;
    ble_hs_cfg.gatts_register_cb = nus_gatt_register_cb;

    /* 存储配置 (bonding 等需要) */
    ble_store_config_init();
}

static void nimble_host_task(void *param)
{
    (void)param;
    ESP_LOGI(TAG, "NimBLE host task started");
    nimble_port_run();
    vTaskDelete(NULL);
}

/* ============================================================
 *  公共 API
 * ============================================================ */

void AX_BLE_Init(const char *device_name)
{
    (void)device_name;  /* 当前固定使用 DEVICE_NAME */

    /* NVS 初始化 */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "nvs_flash_init failed: %d", ret);
        return;
    }

    /* NimBLE 栈初始化 */
    ret = nimble_port_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "nimble_port_init failed: %d", ret);
        return;
    }

    /* GAP 服务 */
    ble_svc_gap_init();
    ble_svc_gap_device_name_set(DEVICE_NAME);

    /* GATT 服务 */
    ble_svc_gatt_init();

    /* Host 配置 */
    nimble_host_config_init();

    /* 启动 NimBLE Host 任务 */
    BaseType_t rc = xTaskCreate(nimble_host_task, "nimble_host",
                                4 * 1024, NULL, 5, NULL);
    if (rc != pdPASS) {
        ESP_LOGE(TAG, "failed to create NimBLE host task");
        return;
    }

    /* 清零全局数据 */
    memset(&ax_ble_joystick, 0, sizeof(ax_ble_joystick));
    memset(&ax_ble_slider,   0, sizeof(ax_ble_slider));
    memset(&ax_ble_key,      0, sizeof(ax_ble_key));
    pkt_idx = 0;

    ESP_LOGI(TAG, "BLE NUS initialized successfully");
}

int AX_BLE_Send(const uint8_t *data, uint16_t len)
{
    if (conn_handle == BLE_HS_CONN_HANDLE_NONE || !tx_notify_enabled) {
        return -1;
    }

    struct os_mbuf *om = ble_hs_mbuf_from_flat(data, len);
    if (om == NULL) {
        return -2;
    }

    int rc = ble_gattc_notify_custom(conn_handle, nus_tx_val_handle, om);
    return rc;
}

/* ============================================================
 *  数据包解析器
 * ============================================================
 *
 *  输入: "joystick,100,200,-50,0"  (已去掉 [ 和 ])
 *        或 "j,100,200,-50,0"       (短格式)
 */

#define MAX_TOKENS  10

static uint8_t tokenize(const char *str, uint8_t len,
                         const char *tokens[], uint8_t max_tok)
{
    uint8_t count = 0;
    uint8_t start = 0;

    for (uint8_t i = 0; i <= len && count < max_tok; i++) {
        if (i == len || str[i] == ',') {
            if (i > start) {
                /* 去掉前后空白 */
                while (start < i && str[start] == ' ') start++;
                uint8_t end = i;
                while (end > start && str[end - 1] == ' ') end--;

                if (end > start) {
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

/* 不区分大小写的字符串比较 */
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

    /* 按逗号分割 */
    const char *tokens[MAX_TOKENS];
    uint8_t n = tokenize(str, len, tokens, MAX_TOKENS);
    if (n == 0) return;

    const char *cmd = tokens[0];

    /* ---- 摇杆 ---- */
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

    /* ---- 滑杆 ---- */
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

    /* ---- 按键 ---- */
    if (streq_nocase(cmd, "key") || streq_nocase(cmd, "k")) {
        if (n >= 3) {
            ax_ble_key.name = (int16_t)atoi(tokens[1]);
            ax_ble_key.is_down = streq_nocase(tokens[2], "down") ||
                                 streq_nocase(tokens[2], "d");
            ax_ble_key.valid = true;
            ESP_LOGI(TAG, "Key: name=%d %s",
                     ax_ble_key.name,
                     ax_ble_key.is_down ? "DOWN" : "UP");
        }
        return;
    }

    /* display / plot — 暂忽略 */
    ESP_LOGD(TAG, "Unhandled: [%.*s]", len, str);
}
