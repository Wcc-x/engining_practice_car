/**
 * @file    ax_ble.c
 * @brief   BLE 蓝牙接收 — NimBLE GATT Server (NUS) + 江协科技数据包解析
 *
 * 基于 ESP-IDF v6.0.1 NimBLE 实现 Nordic UART Service (NUS) 外设角色。
 * 小程序作为 GATT Client 连接后，通过 NUS RX Characteristic 写入数据包，
 * ESP32 解析数据包并提取控制指令（摇杆/滑杆/按键）。
 *
 * 数据包格式:
 *   [joystick,vx,vy,servo1,servo2]  或 [j,vx,vy,servo1,servo2]  摇杆+舵机
 *   [slider,name,value]             或 [s,name,value]            滑杆
 *   [key,name,down/up]              或 [k,name,d/u]              按键
 *
 * NUS UUID (Nordic UART Service 标准):
 *   Service:      6E400001-B5A3-F393-E0A9-E50E24DCCA9E
 *   RX = Write:   6E400002-B5A3-F393-E0A9-E50E24DCCA9E
 *   TX = Notify:  6E400003-B5A3-F393-E0A9-E50E24DCCA9E
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
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

static const char *TAG = "AX_BLE";

/* NimBLE store config — declared in ble_store_config.c (no public header) */
extern void ble_store_config_init(void);

/*   
 *  设备名称
 *    */
#define DEVICE_NAME  "XTARK-ESP32"

/*   
 *  NUS UUID 128-bit (Nordic UART Service, 小端序)
 *
 *  标准: 6E400001-B5A3-F393-E0A9-E50E24DCCA9E
 *  NimBLE BLE_UUID128_INIT 按 LSB 顺序排列:
 *      9E CA DC 24 0E E5 A9 E0 93 F3 A3 B5 01 00 40 6E
 *    */

static const ble_uuid128_t nus_svc_uuid = BLE_UUID128_INIT(
    0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
    0x93, 0xF3, 0xA3, 0xB5, 0x01, 0x00, 0x40, 0x6E
);

/* RX (phone→ESP, Write) — 第2字节改为 02 */
static const ble_uuid128_t nus_rx_uuid = BLE_UUID128_INIT(
    0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
    0x93, 0xF3, 0xA3, 0xB5, 0x02, 0x00, 0x40, 0x6E
);

/* TX (ESP→phone, Notify) — 第2字节改为 03 */
static const ble_uuid128_t nus_tx_uuid = BLE_UUID128_INIT(
    0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
    0x93, 0xF3, 0xA3, 0xB5, 0x03, 0x00, 0x40, 0x6E
);

/*   
 *  GATT 句柄 & 运行时状态
 *    */
static uint16_t nus_rx_val_handle;
static uint16_t nus_tx_val_handle;
static uint16_t conn_handle = BLE_HS_CONN_HANDLE_NONE;
static bool     tx_notify_enabled;

/*   
 *  全局数据 (对外暴露, 见 ax_ble.h)
 *    */
AX_BLE_Joystick  ax_ble_joystick;
AX_BLE_Slider    ax_ble_slider;
AX_BLE_Key       ax_ble_key;
bool             ax_ble_connected;

#define RX_BUF_SIZE  512
uint8_t  ax_ble_rx_buf[RX_BUF_SIZE];
uint16_t ax_ble_rx_len;

/* 数据包累积 */
static char   pkt_buf[256];
static uint8_t pkt_idx;

/*   
 *  前向声明
 *    */
static int  nus_gap_event_handler(struct ble_gap_event *event, void *arg);
static int  nus_gatt_access_cb(uint16_t c_handle, uint16_t attr_handle,
                                struct ble_gatt_access_ctxt *ctxt, void *arg);
static void nus_start_advertising(void);
static void ax_ble_parse_packet(const char *str, uint8_t len);
static void nus_on_stack_sync(void);
static void nus_on_stack_reset(int reason);
void        nus_gatt_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg);

/*   
 *  GATT 服务表 — Nordic UART Service
 *
 *  注意: CCCD 由 NimBLE 自动添加, 不要手动声明!
 *        其句柄 = nus_tx_val_handle + 1
 *    */
static const struct ble_gatt_svc_def nus_gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &nus_svc_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                /* RX — Write (phone→ESP) */
                .uuid       = &nus_rx_uuid.u,
                .access_cb  = nus_gatt_access_cb,
                .flags      = BLE_GATT_CHR_F_WRITE,
                .val_handle = &nus_rx_val_handle,
            },
            {
                /* TX — Notify (ESP→phone), CCCD 自动添加在 val_handle+1 */
                .uuid       = &nus_tx_uuid.u,
                .access_cb  = nus_gatt_access_cb,
                .flags      = BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &nus_tx_val_handle,
            },
            { 0 }
        },
    },
    { 0 }
};

/*   
 *  GATT 注册回调
 *    */
void nus_gatt_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg)
{
    (void)arg;
    char buf[BLE_UUID_STR_LEN];

    switch (ctxt->op) {
    case BLE_GATT_REGISTER_OP_SVC:
        ESP_LOGD(TAG, "registered service %s handle=%d",
                 ble_uuid_to_str(ctxt->svc.svc_def->uuid, buf),
                 ctxt->svc.handle);
        break;
    case BLE_GATT_REGISTER_OP_CHR:
        ESP_LOGD(TAG, "registered characteristic %s def_handle=%d val_handle=%d",
                 ble_uuid_to_str(ctxt->chr.chr_def->uuid, buf),
                 ctxt->chr.def_handle, ctxt->chr.val_handle);
        break;
    case BLE_GATT_REGISTER_OP_DSC:
        ESP_LOGD(TAG, "registered descriptor %s handle=%d",
                 ble_uuid_to_str(ctxt->dsc.dsc_def->uuid, buf),
                 ctxt->dsc.handle);
        break;
    default:
        break;
    }
}

/*   
 *  mbuf → flat buffer 辅助函数
 *    */
static int gatt_write_to_buf(struct os_mbuf *om, uint16_t max_len,
                              void *dst, uint16_t *out_len)
{
    uint16_t om_len = OS_MBUF_PKTLEN(om);
    if (om_len > max_len) {
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }

    int rc = ble_hs_mbuf_to_flat(om, dst, max_len, out_len);
    if (rc != 0) {
        return BLE_ATT_ERR_UNLIKELY;
    }
    return 0;
}

/*   
 *  GATT 访问回调
 *    attr_handle == nus_rx_val_handle         → RX Write
 *    attr_handle == nus_tx_val_handle + 1     → CCCD Write (NimBLE auto)
 *    */
static int nus_gatt_access_cb(uint16_t c_handle, uint16_t attr_handle,
                               struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)arg;
    (void)c_handle;

    /* ---- RX Write: 手机→ESP32 ---- */
    if (attr_handle == nus_rx_val_handle &&
        ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {

        uint16_t om_len = OS_MBUF_PKTLEN(ctxt->om);
        if (om_len == 0 || om_len >= RX_BUF_SIZE) {
            return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
        }

        uint16_t len;
        int rc = gatt_write_to_buf(ctxt->om, RX_BUF_SIZE - 1, ax_ble_rx_buf, &len);
        if (rc != 0) return rc;

        ax_ble_rx_len = len;
        ax_ble_rx_buf[len] = '\0';

        /* 逐字节累积到数据包缓冲区 */
        for (uint16_t i = 0; i < len; i++) {
            char ch = (char)ax_ble_rx_buf[i];

            if (ch == '[') {
                pkt_idx = 0;
                pkt_buf[pkt_idx++] = ch;
            } else if (pkt_idx > 0 && pkt_idx < sizeof(pkt_buf) - 1) {
                pkt_buf[pkt_idx++] = ch;
                if (ch == ']') {
                    pkt_buf[pkt_idx] = '\0';
                    ax_ble_parse_packet(pkt_buf + 1, pkt_idx - 2);
                    pkt_idx = 0;
                }
            }
        }

        return 0;
    }

    /* ---- CCCD Write: 手机订阅/取消 Notify ---- */
    /* NimBLE 自动添加的 CCCD, handle = val_handle + 1 */
    if (attr_handle == (uint16_t)(nus_tx_val_handle + 1) &&
        ctxt->op == BLE_GATT_ACCESS_OP_WRITE_DSC) {

        uint8_t cccd[2];
        uint16_t len;
        int rc = gatt_write_to_buf(ctxt->om, sizeof(cccd), cccd, &len);
        if (rc != 0) return rc;

        uint16_t val = (len >= 2) ? ((uint16_t)cccd[1] << 8) | cccd[0] : cccd[0];
        tx_notify_enabled = (val & 0x0001) != 0;
        ESP_LOGI(TAG, "TX notify %s", tx_notify_enabled ? "ENABLED" : "DISABLED");
        return 0;
    }

    return BLE_ATT_ERR_UNLIKELY;
}

/*   
 *  GAP 事件处理
 *    */
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
        ESP_LOGI(TAG, "Subscribe: conn=%d attr=%d cur_notify=%d cur_indicate=%d",
                 event->subscribe.conn_handle,
                 event->subscribe.attr_handle,
                 event->subscribe.cur_notify,
                 event->subscribe.cur_indicate);
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

/*   
 *  广播
 *    */
static void nus_start_advertising(void)
{
    struct ble_hs_adv_fields adv_fields = {0};
    struct ble_gap_adv_params adv_params = {0};
    int rc;

    adv_fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    adv_fields.name = (const uint8_t *)DEVICE_NAME;
    adv_fields.name_len = strlen(DEVICE_NAME);
    adv_fields.name_is_complete = 1;

    rc = ble_gap_adv_set_fields(&adv_fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "failed to set adv fields: %d", rc);
        return;
    }

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

/*   
 *  NimBLE 栈回调
 *    */
static void nus_on_stack_sync(void)
{
    int rc;

    rc = ble_hs_util_ensure_addr(0);
    if (rc != 0) {
        ESP_LOGE(TAG, "no BT address available");
        return;
    }

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

    nus_start_advertising();
}

static void nus_on_stack_reset(int reason)
{
    ESP_LOGI(TAG, "NimBLE stack reset, reason=%d", reason);
}

/*   
 *  NimBLE Host 配置 & 任务
 *    */
static void nimble_host_config_init(void)
{
    ble_hs_cfg.reset_cb             = nus_on_stack_reset;
    ble_hs_cfg.sync_cb              = nus_on_stack_sync;
    ble_hs_cfg.gatts_register_cb    = nus_gatt_register_cb;
    ble_store_config_init();
}

static void nimble_host_task(void *param)
{
    (void)param;
    ESP_LOGI(TAG, "NimBLE host task started");
    nimble_port_run();
    vTaskDelete(NULL);
}

/*   
 *  公共 API
 *    */

void AX_BLE_Init(const char *device_name)
{
    (void)device_name;

    /* NVS */
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

    /* NimBLE 栈 */
    ret = nimble_port_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "nimble_port_init failed: %d", ret);
        return;
    }

    /* GAP + GATT 服务 */
    ble_svc_gap_init();
    ble_svc_gap_device_name_set(DEVICE_NAME);
    ble_svc_gatt_init();

    /* Host 配置 */
    nimble_host_config_init();

    /* 启动 NimBLE 任务 */
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
    tx_notify_enabled = false;
    conn_handle = BLE_HS_CONN_HANDLE_NONE;
    ax_ble_connected = false;

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

    int rc = ble_gatts_notify_custom(conn_handle, nus_tx_val_handle, om);
    return rc;
}

/*   
 *  数据包解析器
 *    */

#define MAX_TOKENS  10

static uint8_t tokenize(const char *str, uint8_t len,
                         const char *tokens[], uint8_t max_tok)
{
    uint8_t count = 0, start = 0;

    for (uint8_t i = 0; i <= len && count < max_tok; i++) {
        if (i == len || str[i] == ',') {
            if (i > start) {
                while (start < i && str[start] == ' ') start++;
                uint8_t end = i;
                while (end > start && str[end - 1] == ' ') end--;

                if (end > start) {
                    static char tok_buf[MAX_TOKENS][32];
                    uint8_t n = (end - start) < 31 ? (end - start) : 31;
                    memcpy(tok_buf[count], str + start, n);
                    tok_buf[count][n] = '\0';
                    tokens[count] = tok_buf[count];
                    count++;
                }
            }
            start = i + 1;
        }
    }
    return count;
}

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

    while (len > 0 && *str == ' ') { str++; len--; }
    if (len == 0) return;

    const char *tokens[MAX_TOKENS];
    uint8_t n = tokenize(str, len, tokens, MAX_TOKENS);
    if (n == 0) return;

    const char *cmd = tokens[0];

    /* ---- 摇杆+舵机: [joystick,vx,vy,servo1,servo2] / [j,vx,vy,servo1,servo2] ---- */
    if (streq_nocase(cmd, "joystick") || streq_nocase(cmd, "j")) {
        if (n >= 5) {
            ax_ble_joystick.vx           = (int16_t)atoi(tokens[1]);
            ax_ble_joystick.vy           = (int16_t)atoi(tokens[2]);
            ax_ble_joystick.servo1_angle = (int16_t)atoi(tokens[3]);
            ax_ble_joystick.servo2_angle = (int16_t)atoi(tokens[4]);
            ax_ble_joystick.valid        = true;
            ESP_LOGI(TAG, "Joystick: Vx=%d Vy=%d S1=%d S2=%d",
                     ax_ble_joystick.vx, ax_ble_joystick.vy,
                     ax_ble_joystick.servo1_angle, ax_ble_joystick.servo2_angle);
        }
        return;
    }

    /* ---- 滑杆: [slider,name,value] / [s,name,value] ---- */
    if (streq_nocase(cmd, "slider") || streq_nocase(cmd, "s")) {
        if (n >= 3) {
            ax_ble_slider.name  = (int16_t)atoi(tokens[1]);
            ax_ble_slider.value = (int16_t)atoi(tokens[2]);
            ax_ble_slider.valid = true;
            ESP_LOGI(TAG, "Slider: %d = %d",
                     ax_ble_slider.name, ax_ble_slider.value);
        }
        return;
    }

    /* ---- 按键: [key,name,down/up] / [k,name,d/u] ---- */
    if (streq_nocase(cmd, "key") || streq_nocase(cmd, "k")) {
        if (n >= 3) {
            ax_ble_key.name = (int16_t)atoi(tokens[1]);
            ax_ble_key.is_down = streq_nocase(tokens[2], "down") ||
                                 streq_nocase(tokens[2], "d");
            ax_ble_key.valid = true;
            ESP_LOGI(TAG, "Key: %d %s",
                     ax_ble_key.name,
                     ax_ble_key.is_down ? "DOWN" : "UP");
        }
        return;
    }

    ESP_LOGD(TAG, "Unhandled: [%.*s]", len, str);
}
