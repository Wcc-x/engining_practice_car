// #include "ax_ble.h"

#include "ax_ble.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "esp_log.h"
#include "nvs_flash.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "host/util/util.h"
#include "nimble/ble.h"
#include "nimble/nimble_port.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "nimble/nimble_port_freertos.h"

static const char *TAG = "AX_BLE";

static uint8_t own_addr_type;
extern void ble_store_config_init(void);

#define DEFAULT_DEVICE_NAME  "XTARK-ESP32"
#define DEVICE_NAME_MAX_LEN  31
static char g_device_name[DEVICE_NAME_MAX_LEN + 1] = DEFAULT_DEVICE_NAME;

#define BLE_APPEARANCE_GENERIC_ROBOT  320

static const ble_uuid128_t nus_svc_uuid = BLE_UUID128_INIT(
    0x9E,0xCA,0xDC,0x24,0x0E,0xE5,0xA9,0xE0,
    0x93,0xF3,0xA3,0xB5,0x01,0x00,0x40,0x6E
);
static const ble_uuid128_t nus_rx_uuid = BLE_UUID128_INIT(
    0x9E,0xCA,0xDC,0x24,0x0E,0xE5,0xA9,0xE0,
    0x93,0xF3,0xA3,0xB5,0x02,0x00,0x40,0x6E
);
static const ble_uuid128_t nus_tx_uuid = BLE_UUID128_INIT(
    0x9E,0xCA,0xDC,0x24,0x0E,0xE5,0xA9,0xE0,
    0x93,0xF3,0xA3,0xB5,0x03,0x00,0x40,0x6E
);

static uint16_t nus_rx_val_handle;
static uint16_t nus_tx_val_handle;
static uint16_t conn_handle = BLE_HS_CONN_HANDLE_NONE;
static bool     tx_notify_enabled;

volatile AX_BLE_Joystick  ax_ble_joystick;
volatile AX_BLE_Slider    ax_ble_slider;
volatile AX_BLE_Key       ax_ble_key;
volatile bool             ax_ble_connected;
volatile uint32_t         ax_ble_gap_evt_cnt;
volatile uint16_t         ax_ble_gap_last_evt;

#define RX_BUF_SIZE 512
volatile uint8_t  ax_ble_rx_buf[RX_BUF_SIZE];
volatile uint16_t ax_ble_rx_len;

static char    pkt_buf[256];
static uint8_t pkt_idx;

/* 前向声明 */
static int     nus_gap_event_handler(struct ble_gap_event *event, void *arg);
static int     nus_gatt_access_cb(uint16_t conn, uint16_t attr,
                                   struct ble_gatt_access_ctxt *ctxt, void *arg);
static void    nus_start_advertising(void);
static void    ax_ble_parse_packet(const char *str, uint8_t len);
static uint8_t tokenize(const char *str, uint8_t len,
                         const char *tokens[], uint8_t max,
                         char (*buf)[32]);
void           nus_gatt_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg);

/* ----------------------------------------------------------------
 *  GATT 服务表
 * ---------------------------------------------------------------- */
static const struct ble_gatt_svc_def nus_gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &nus_svc_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid      = &nus_rx_uuid.u,
                .access_cb = nus_gatt_access_cb,
                .flags     = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
                .val_handle= &nus_rx_val_handle,
            },
            {
                .uuid      = &nus_tx_uuid.u,
                .access_cb = nus_gatt_access_cb,
                .flags     = BLE_GATT_CHR_F_NOTIFY,
                .val_handle= &nus_tx_val_handle,
            },
            { 0 }
        },
    },
    { 0 }
};

void nus_gatt_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg)
{
    (void)arg;
    char buf[BLE_UUID_STR_LEN];
    switch (ctxt->op) {
    case BLE_GATT_REGISTER_OP_SVC:
        ESP_LOGI(TAG, "SVC  registered: %s handle=%d",
                 ble_uuid_to_str(ctxt->svc.svc_def->uuid, buf), ctxt->svc.handle);
        break;
    case BLE_GATT_REGISTER_OP_CHR:
        ESP_LOGI(TAG, "CHR  registered: %s val_handle=%d",
                 ble_uuid_to_str(ctxt->chr.chr_def->uuid, buf), ctxt->chr.val_handle);
        break;
    case BLE_GATT_REGISTER_OP_DSC:
        ESP_LOGI(TAG, "DSC  registered: %s handle=%d",
                 ble_uuid_to_str(ctxt->dsc.dsc_def->uuid, buf), ctxt->dsc.handle);
        break;
    default: break;
    }
}

static int gatt_write_to_buf(struct os_mbuf *om, uint16_t max_len,
                              void *dst, uint16_t *out_len)
{
    if (OS_MBUF_PKTLEN(om) > max_len) return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    return ble_hs_mbuf_to_flat(om, dst, max_len, out_len) ? BLE_ATT_ERR_UNLIKELY : 0;
}

static int nus_gatt_access_cb(uint16_t conn, uint16_t attr,
                               struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)conn; (void)arg;

    if (attr == nus_rx_val_handle &&
        ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {

        uint16_t om_len = OS_MBUF_PKTLEN(ctxt->om);
        if (om_len == 0 || om_len >= RX_BUF_SIZE)
            return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;

        uint16_t len;
        int rc = gatt_write_to_buf(ctxt->om, RX_BUF_SIZE-1,
                                   (void*)ax_ble_rx_buf, &len);
        if (rc) return rc;

        ax_ble_rx_len = len;
        ax_ble_rx_buf[len] = '\0';

        for (uint16_t i = 0; i < len; i++) {
            char ch = (char)ax_ble_rx_buf[i];
            if (ch == '[') {
                pkt_idx = 0;
                pkt_buf[pkt_idx++] = ch;
            } else if (pkt_idx > 0 && pkt_idx < (uint8_t)(sizeof(pkt_buf)-1)) {
                pkt_buf[pkt_idx++] = ch;
                if (ch == ']') {
                    pkt_buf[pkt_idx] = '\0';
                    ax_ble_parse_packet(pkt_buf+1, pkt_idx-2);
                    pkt_idx = 0;
                }
            }
        }
        return 0;
    }
    return 0;
}

static int nus_gap_event_handler(struct ble_gap_event *event, void *arg)
{
    (void)arg;
    ax_ble_gap_evt_cnt++;
    ax_ble_gap_last_evt = (uint16_t)event->type;
    printf("[GAP] #%lu type=%d\n", (unsigned long)ax_ble_gap_evt_cnt, event->type);

    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            ESP_LOGI(TAG, "Connected! handle=%d", event->connect.conn_handle);
            conn_handle = event->connect.conn_handle;
            ax_ble_connected = true;
        } else {
            nus_start_advertising();
        }
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "Disconnected reason=%d", event->disconnect.reason);
        conn_handle = BLE_HS_CONN_HANDLE_NONE;
        ax_ble_connected = false;
        tx_notify_enabled = false;
        pkt_idx = 0;
        memset((void*)&ax_ble_joystick, 0, sizeof(ax_ble_joystick));
        memset((void*)&ax_ble_slider,   0, sizeof(ax_ble_slider));
        memset((void*)&ax_ble_key,      0, sizeof(ax_ble_key));
        nus_start_advertising();
        return 0;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        nus_start_advertising();
        return 0;

    case BLE_GAP_EVENT_SUBSCRIBE:
        if (event->subscribe.attr_handle == nus_tx_val_handle) {
            tx_notify_enabled = (event->subscribe.cur_notify != 0);
            ESP_LOGI(TAG, "TX notify %s", tx_notify_enabled ? "ENABLED" : "DISABLED");
        }
        return 0;

    case BLE_GAP_EVENT_MTU:
        ESP_LOGI(TAG, "MTU=%d", event->mtu.value);
        return 0;

    default:
        return 0;
    }
}

static void nus_start_advertising(void)
{
    struct ble_hs_adv_fields adv = {0};
    struct ble_hs_adv_fields rsp = {0};
    struct ble_gap_adv_params par = {0};

    adv.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    adv.uuids128 = &nus_svc_uuid;
    adv.num_uuids128 = 1;
    adv.uuids128_is_complete = 1;
    adv.name = (const uint8_t*)g_device_name;
    adv.name_len = strlen(g_device_name) > 7 ? 7 : strlen(g_device_name);
    adv.name_is_complete = 0;

    if (ble_gap_adv_set_fields(&adv) != 0) {
        ESP_LOGE(TAG, "set adv fields failed"); return;
    }

    rsp.name = (const uint8_t*)g_device_name;
    rsp.name_len = strlen(g_device_name);
    rsp.name_is_complete = 1;
    rsp.tx_pwr_lvl_is_present = 1;
    rsp.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;
    ble_gap_adv_rsp_set_fields(&rsp);

    par.conn_mode = BLE_GAP_CONN_MODE_UND;
    par.disc_mode = BLE_GAP_DISC_MODE_GEN;
    par.itvl_min  = BLE_GAP_ADV_ITVL_MS(30);
    par.itvl_max  = BLE_GAP_ADV_ITVL_MS(60);

    if (ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER,
                          &par, nus_gap_event_handler, NULL) != 0) {
        ESP_LOGE(TAG, "start adv failed"); return;
    }
    ESP_LOGI(TAG, "Advertising as '%s'", g_device_name);
}

/* ----------------------------------------------------------------
 *  ★ sync_cb: 地址推导完成后直接开广播
 *    服务已在 Host 启动前注册，这里无需再注册
 * ---------------------------------------------------------------- */
static void nus_on_stack_sync(void)
{
    int rc = ble_hs_util_ensure_addr(0);
    if (rc) { ESP_LOGE(TAG, "no BT address"); return; }

    rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc) { ESP_LOGE(TAG, "infer addr type failed: %d", rc); return; }

    ESP_LOGI(TAG, "Stack synced, starting advertising...");
    nus_start_advertising();
}

static void nus_on_stack_reset(int reason)
{
    ESP_LOGW(TAG, "NimBLE reset reason=%d", reason);
}

static void nimble_host_task(void *param)
{
    (void)param;
    ESP_LOGI(TAG, "NimBLE host task started");
    nimble_port_run();
    nimble_port_freertos_deinit();
}

/* ----------------------------------------------------------------
 *  AX_BLE_Init — 服务在 Host 启动前注册 ★
 * ---------------------------------------------------------------- */
void AX_BLE_Init(const char *device_name)
{
    if (device_name && device_name[0]) {
        size_t n = strlen(device_name);
        if (n > DEVICE_NAME_MAX_LEN) n = DEVICE_NAME_MAX_LEN;
        memcpy(g_device_name, device_name, n);
        g_device_name[n] = '\0';
    }

    /* NVS */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* NimBLE 端口初始化 */
    ESP_ERROR_CHECK(nimble_port_init());

    /* GAP / GATT 基础服务 */
    ble_svc_gap_init();
    ble_svc_gap_device_name_set(g_device_name);
    ble_svc_gap_device_appearance_set(BLE_APPEARANCE_GENERIC_ROBOT);
    ble_svc_gatt_init();

    /* Host 回调 */
    ble_hs_cfg.reset_cb          = nus_on_stack_reset;
    ble_hs_cfg.sync_cb           = nus_on_stack_sync;
    ble_hs_cfg.gatts_register_cb = nus_gatt_register_cb;
    ble_store_config_init();

    /* ★ 在 Host 任务启动前注册 NUS 服务 ★ */
    printf(">>> STEP1: calling gatts_count_cfg\n");
    int rc = ble_gatts_count_cfg(nus_gatt_svcs);
    printf(">>> STEP2: gatts_count_cfg rc=%d\n", rc);
    if (rc != 0) { ESP_LOGE(TAG, "gatts_count_cfg failed: %d", rc); return; }

    rc = ble_gatts_add_svcs(nus_gatt_svcs);
    printf(">>> STEP3: gatts_add_svcs rc=%d\n", rc);
    if (rc != 0) { ESP_LOGE(TAG, "gatts_add_svcs failed: %d", rc); return; }
    printf(">>> STEP4: NUS registered OK\n");

    /* 启动 NimBLE Host 任务 — sync_cb 中启动广播 */
    nimble_port_freertos_init(nimble_host_task);

    /* 清零 */
    memset((void*)&ax_ble_joystick, 0, sizeof(ax_ble_joystick));
    memset((void*)&ax_ble_slider,   0, sizeof(ax_ble_slider));
    memset((void*)&ax_ble_key,      0, sizeof(ax_ble_key));
    pkt_idx           = 0;
    tx_notify_enabled = false;
    conn_handle       = BLE_HS_CONN_HANDLE_NONE;
    ax_ble_connected  = false;
    ax_ble_gap_evt_cnt  = 0;
    ax_ble_gap_last_evt = 0;

    ESP_LOGI(TAG, "BLE NUS initialized as '%s'", g_device_name);
}

int AX_BLE_Send(const uint8_t *data, uint16_t len)
{
    if (conn_handle == BLE_HS_CONN_HANDLE_NONE || !tx_notify_enabled) return -1;
    struct os_mbuf *om = ble_hs_mbuf_from_flat(data, len);
    if (!om) return -2;
    return ble_gatts_notify_custom(conn_handle, nus_tx_val_handle, om) ? -3 : 0;
}

/* ----------------------------------------------------------------
 *  数据包解析
 * ---------------------------------------------------------------- */
#define MAX_TOKENS 10

static uint8_t tokenize(const char *str, uint8_t len,
                         const char *tokens[], uint8_t max,
                         char (*buf)[32])
{
    uint8_t count = 0, start = 0;
    for (uint8_t i = 0; i <= len && count < max; i++) {
        if (i == len || str[i] == ',') {
            if (i > start) {
                while (start < i && str[start] == ' ') start++;
                uint8_t end = i;
                while (end > start && str[end-1] == ' ') end--;
                if (end > start) {
                    uint8_t n = (end-start) < 31 ? (end-start) : 31;
                    memcpy(buf[count], str+start, n);
                    buf[count][n] = '\0';
                    tokens[count] = buf[count];
                    count++;
                }
            }
            start = i+1;
        }
    }
    return count;
}

static bool streq_nocase(const char *a, const char *b)
{
    while (*a && *b) {
        char ca = *a, cb = *b;
        if (ca>='A'&&ca<='Z') ca+=32;
        if (cb>='A'&&cb<='Z') cb+=32;
        if (ca!=cb) return false;
        a++; b++;
    }
    return !*a && !*b;
}

static void ax_ble_parse_packet(const char *str, uint8_t len)
{
    if (!len || !str) return;
    while (len > 0 && *str == ' ') { str++; len--; }
    if (!len) return;

    const char *tokens[MAX_TOKENS];
    char tok_buf[MAX_TOKENS][32];
    uint8_t n = tokenize(str, len, tokens, MAX_TOKENS, tok_buf);
    if (!n) return;

    const char *cmd = tokens[0];

    if (streq_nocase(cmd,"joystick") || streq_nocase(cmd,"j")) {
        if (n >= 5) {
            ax_ble_joystick.vx           = (int16_t)atoi(tokens[1]);
            ax_ble_joystick.vy           = (int16_t)atoi(tokens[2]);
            ax_ble_joystick.servo1_angle = (int16_t)atoi(tokens[3]);
            ax_ble_joystick.servo2_angle = (int16_t)atoi(tokens[4]);
            ax_ble_joystick.valid        = true;
            ESP_LOGI(TAG, "Joystick => Vx=%d Vy=%d RX=%d RY=%d",
                     ax_ble_joystick.vx, ax_ble_joystick.vy,
                     ax_ble_joystick.servo1_angle, ax_ble_joystick.servo2_angle);
        }
        return;
    }

    if (streq_nocase(cmd,"slider") || streq_nocase(cmd,"s")) {
        if (n >= 3) {
            ax_ble_slider.name  = (int16_t)atoi(tokens[1]);
            ax_ble_slider.value = (int16_t)atoi(tokens[2]);
            ax_ble_slider.valid = true;
            ESP_LOGI(TAG, "Slider => name=%d value=%d",
                     ax_ble_slider.name, ax_ble_slider.value);
        }
        return;
    }

    if (streq_nocase(cmd,"key") || streq_nocase(cmd,"k")) {
        if (n >= 3) {
            ax_ble_key.name    = (int16_t)atoi(tokens[1]);
            ax_ble_key.is_down = streq_nocase(tokens[2],"down") ||
                                 streq_nocase(tokens[2],"d");
            ax_ble_key.valid   = true;
            ESP_LOGI(TAG, "Key => name=%d %s",
                     ax_ble_key.name, ax_ble_key.is_down?"DOWN":"UP");
        }
        return;
    }

    ESP_LOGD(TAG, "Unhandled: [%.*s]", len, str);
}