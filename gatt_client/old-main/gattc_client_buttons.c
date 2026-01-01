/*
 * BLE GATT Client (Bluedroid) - ESP-IDF 5.5.1
 * - Connects to a peripheral advertising a given name (REMOTE_DEVICE_NAME)
 * - Discovers service (REMOTE_SERVICE_UUID) and streaming notify characteristic (128-bit UUID)
 * - Stores CCCD handle for the notify characteristic
 * - Uses two GPIO buttons to ENABLE/DISABLE notifications by writing 0x0001/0x0000 to CCCD
 *
 * NOTE:
 *  - Button GPIOs depend on your board. Update BTN_ENABLE_GPIO / BTN_DISABLE_GPIO accordingly.
 *  - This file is meant to be used as main.c of your GATTC project.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"

#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_gatt_defs.h"
#include "esp_gatt_common_api.h"

#include "driver/gpio.h"

// --------------------- CONFIG ---------------------

#define PROFILE_NUM                 1
#define PROFILE_A_APP_ID            0

// Peripheral advertised name to connect to
#define REMOTE_DEVICE_NAME          "DPPS-DTS"

// Service UUID (16-bit) used by your server (you used 0x1815 for AUTO-IO in your server)
#define REMOTE_SERVICE_UUID         0x1815

// 128-bit UUID for the STREAM characteristic (must match server stream_chr_uuid)
static esp_bt_uuid_t remote_filter_char_uuid = {
    .len = ESP_UUID_LEN_128,
    .uuid = {.uuid128 = {
        0x23, 0xd1, 0xbc, 0xea, 0x5f, 0x78, 0x23, 0x15,
        0xde, 0xef, 0x12, 0x12, 0x25, 0x15, 0x00, 0x01
    }},
};

// CCCD UUID
static esp_bt_uuid_t notify_descr_uuid = {
    .len = ESP_UUID_LEN_16,
    .uuid = {.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG,},
};

// Button pins (ACTIVE LOW, internal pull-up enabled)
#if CONFIG_IDF_TARGET_ESP32C6
// Typical ESP32-C6 devkits: BOOT is often GPIO9. Pick another free GPIO for disable.
#define BTN_ENABLE_GPIO             9
#define BTN_DISABLE_GPIO            8
#else
#define BTN_ENABLE_GPIO             0
#define BTN_DISABLE_GPIO            2
#endif

#define BTN_POLL_MS                 20
#define BTN_DEBOUNCE_MS             40

// --------------------- RF tuning (optional) ---------------------

static const char *GATTC_TAG = "GATTC-STREAM";

#define ESP_PWR_LVL                 ESP_PWR_LVL_N0  // keep as in your snippet (change to P9 if you want more TX power)

static void ble_rf_tune_before_adv(void)
{
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, ESP_PWR_LVL);
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV,     ESP_PWR_LVL);
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_SCAN,    ESP_PWR_LVL);

    // Prefer coded + 1M (peer decides final)
    esp_ble_gap_set_preferred_default_phy(
        ESP_BLE_GAP_PHY_CODED_PREF_MASK | ESP_BLE_GAP_PHY_1M_PREF_MASK,
        ESP_BLE_GAP_PHY_CODED_PREF_MASK | ESP_BLE_GAP_PHY_1M_PREF_MASK
    );
}

static void ble_rf_tune_on_connect(esp_bd_addr_t peer_bda)
{
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_CONN_HDL0, ESP_PWR_LVL);

    // Prefer S8 coded
    esp_err_t err = esp_ble_gap_set_preferred_phy(
        peer_bda,
        0,
        ESP_BLE_GAP_PHY_CODED_PREF_MASK | ESP_BLE_GAP_PHY_1M_PREF_MASK,
        ESP_BLE_GAP_PHY_CODED_PREF_MASK | ESP_BLE_GAP_PHY_1M_PREF_MASK,
        ESP_BLE_GAP_PHY_OPTIONS_PREF_S8_CODING
    );

    if (err != ESP_OK) {
        ESP_LOGW(GATTC_TAG, "set_preferred_phy failed: %s", esp_err_to_name(err));
    }

    err = esp_ble_gap_read_phy(peer_bda);
    if (err != ESP_OK) {
        ESP_LOGW(GATTC_TAG, "read_phy failed: %s", esp_err_to_name(err));
    }
}

// --------------------- BLE scan params ---------------------

static esp_ble_scan_params_t ble_scan_params = {
    .scan_type              = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type          = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy     = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval          = 0x50,
    .scan_window            = 0x30,
    .scan_duplicate         = BLE_SCAN_DUPLICATE_ENABLE
};

// --------------------- Profile + state ---------------------

struct gattc_profile_inst {
    esp_gattc_cb_t gattc_cb;
    uint16_t gattc_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_start_handle;
    uint16_t service_end_handle;
    uint16_t char_handle;
    uint16_t cccd_handle;
    esp_bd_addr_t remote_bda;
};

static struct gattc_profile_inst gl_profile_tab[PROFILE_NUM];

static bool connect_ok      = false;
static bool get_server_ok   = false;
static bool notify_enabled  = false;

static esp_gattc_char_elem_t *char_elem_result   = NULL;
static esp_gattc_descr_elem_t *descr_elem_result = NULL;

// --------------------- helpers ---------------------

static bool adv_name_matches(const uint8_t *adv_data, uint8_t adv_data_len, const char *name)
{
    uint8_t len = 0;
    uint8_t *p = esp_ble_resolve_adv_data(adv_data, ESP_BLE_AD_TYPE_NAME_CMPL, &len);
    if (p && len) {
        // len is not null-terminated
        if (strlen(name) == len && memcmp(p, name, len) == 0) {
            return true;
        }
    }
    return false;
}

static void buttons_init(void)
{
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << BTN_ENABLE_GPIO) | (1ULL << BTN_DISABLE_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&io));
}

static esp_err_t write_cccd(uint16_t value)
{
    struct gattc_profile_inst *p = &gl_profile_tab[PROFILE_A_APP_ID];

    if (!connect_ok || !get_server_ok || p->cccd_handle == 0 || p->gattc_if == ESP_GATT_IF_NONE) {
        ESP_LOGW(GATTC_TAG, "CCCD write skipped (connect_ok=%d get_server_ok=%d cccd=0x%04x)",
                 connect_ok, get_server_ok, p->cccd_handle);
        return ESP_FAIL;
    }

    // CCCD is 2 bytes little-endian
    uint16_t le = value;
    esp_err_t err = esp_ble_gattc_write_char_descr(
        p->gattc_if,
        p->conn_id,
        p->cccd_handle,
        sizeof(le),
        (uint8_t *)&le,
        ESP_GATT_WRITE_TYPE_RSP,
        ESP_GATT_AUTH_REQ_NONE
    );

    if (err != ESP_OK) {
        ESP_LOGE(GATTC_TAG, "esp_ble_gattc_write_char_descr failed: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(GATTC_TAG, "CCCD write requested: 0x%04x", value);
    return ESP_OK;
}

static void button_task(void *arg)
{
    buttons_init();

    int prev_en = 1;
    int prev_dis = 1;

    while (1) {
        int en = gpio_get_level(BTN_ENABLE_GPIO);
        int dis = gpio_get_level(BTN_DISABLE_GPIO);

        // Detect falling edge (active low press)
        if (prev_en == 1 && en == 0) {
            vTaskDelay(pdMS_TO_TICKS(BTN_DEBOUNCE_MS));
            if (gpio_get_level(BTN_ENABLE_GPIO) == 0) {
                ESP_LOGI(GATTC_TAG, "BTN_ENABLE pressed -> ENABLE notify");
                write_cccd(0x0001);
            }
        }

        if (prev_dis == 1 && dis == 0) {
            vTaskDelay(pdMS_TO_TICKS(BTN_DEBOUNCE_MS));
            if (gpio_get_level(BTN_DISABLE_GPIO) == 0) {
                ESP_LOGI(GATTC_TAG, "BTN_DISABLE pressed -> DISABLE notify");
                write_cccd(0x0000);
            }
        }

        prev_en = en;
        prev_dis = dis;

        vTaskDelay(pdMS_TO_TICKS(BTN_POLL_MS));
    }
}

// --------------------- GAP / GATTC callbacks ---------------------

static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
static void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);
static void gattc_profile_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);

static void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
    // Store gattc_if once at REG_EVT
    if (event == ESP_GATTC_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            gl_profile_tab[param->reg.app_id].gattc_if = gattc_if;
        } else {
            ESP_LOGE(GATTC_TAG, "gattc app register failed, status %d", param->reg.status);
            return;
        }
    }

    // Dispatch to profile handler
    for (int idx = 0; idx < PROFILE_NUM; idx++) {
        if (gattc_if == ESP_GATT_IF_NONE || gattc_if == gl_profile_tab[idx].gattc_if) {
            if (gl_profile_tab[idx].gattc_cb) {
                gl_profile_tab[idx].gattc_cb(event, gattc_if, param);
            }
        }
    }
}

static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT:
        esp_ble_gap_start_scanning(0);
        break;

    case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
        ESP_LOGI(GATTC_TAG, "Scan start complete, status=%d", param->scan_start_cmpl.status);
        break;

    case ESP_GAP_BLE_SCAN_RESULT_EVT: {
        esp_ble_gap_cb_param_t *scan = param;
        switch (scan->scan_rst.search_evt) {
        case ESP_GAP_SEARCH_INQ_RES_EVT:
            if (adv_name_matches(scan->scan_rst.ble_adv, scan->scan_rst.adv_data_len, REMOTE_DEVICE_NAME)) {
                ESP_LOGI(GATTC_TAG, "Found device '%s', connecting...", REMOTE_DEVICE_NAME);
                esp_ble_gap_stop_scanning();
                esp_ble_gattc_open(gl_profile_tab[PROFILE_A_APP_ID].gattc_if,
                                   scan->scan_rst.bda,
                                   scan->scan_rst.ble_addr_type,
                                   true);
            }
            break;

        case ESP_GAP_SEARCH_INQ_CMPL_EVT:
            ESP_LOGI(GATTC_TAG, "Scan complete");
            break;

        default:
            break;
        }
        break;
    }

    default:
        break;
    }
}

static void gattc_profile_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
    struct gattc_profile_inst *p = &gl_profile_tab[PROFILE_A_APP_ID];

    switch (event) {
    case ESP_GATTC_REG_EVT: {
        ESP_LOGI(GATTC_TAG, "REG_EVT, status=%d, app_id=%d", param->reg.status, param->reg.app_id);
        esp_err_t scan_ret = esp_ble_gap_set_scan_params(&ble_scan_params);
        if (scan_ret) {
            ESP_LOGE(GATTC_TAG, "set scan params error: %s", esp_err_to_name(scan_ret));
        }
        break;
    }

    case ESP_GATTC_CONNECT_EVT: {
        ESP_LOGI(GATTC_TAG, "CONNECT_EVT, conn_id=%d", param->connect.conn_id);
        connect_ok = true;
        p->conn_id = param->connect.conn_id;
        memcpy(p->remote_bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));

        ble_rf_tune_on_connect(p->remote_bda);

        // MTU request (optional)
        esp_ble_gattc_send_mtu_req(gattc_if, param->connect.conn_id);
        break;
    }

    case ESP_GATTC_OPEN_EVT:
        if (param->open.status != ESP_GATT_OK) {
            ESP_LOGE(GATTC_TAG, "OPEN_EVT failed, status %d", param->open.status);
            connect_ok = false;
            esp_ble_gap_start_scanning(0);
            break;
        }
        ESP_LOGI(GATTC_TAG, "OPEN_EVT ok");
        break;

    case ESP_GATTC_CFG_MTU_EVT:
        ESP_LOGI(GATTC_TAG, "MTU configured, mtu=%d", param->cfg_mtu.mtu);
        // Start service discovery
        esp_ble_gattc_search_service(gattc_if, param->cfg_mtu.conn_id, NULL);
        break;

    case ESP_GATTC_SEARCH_RES_EVT: {
        esp_gatt_srvc_id_t *srvc_id = &param->search_res.srvc_id;
        if (srvc_id->id.uuid.len == ESP_UUID_LEN_16 && srvc_id->id.uuid.uuid.uuid16 == REMOTE_SERVICE_UUID) {
            ESP_LOGI(GATTC_TAG, "Found service 0x%04x", REMOTE_SERVICE_UUID);
            get_server_ok = true;
            p->service_start_handle = param->search_res.start_handle;
            p->service_end_handle   = param->search_res.end_handle;
        }
        break;
    }

    case ESP_GATTC_SEARCH_CMPL_EVT:
        if (!get_server_ok) {
            ESP_LOGW(GATTC_TAG, "Service not found, disconnecting");
            esp_ble_gattc_close(gattc_if, p->conn_id);
            break;
        }

        // Discover characteristic by UUID
        uint16_t count = 0;
        esp_ble_gattc_get_attr_count(gattc_if, p->conn_id,
                                     ESP_GATT_DB_CHARACTERISTIC,
                                     p->service_start_handle,
                                     p->service_end_handle,
                                     INVALID_HANDLE,
                                     &count);

        if (count == 0) {
            ESP_LOGE(GATTC_TAG, "No characteristics found");
            break;
        }

        char_elem_result = (esp_gattc_char_elem_t *)malloc(sizeof(esp_gattc_char_elem_t) * count);
        if (!char_elem_result) {
            ESP_LOGE(GATTC_TAG, "malloc char_elem_result failed");
            break;
        }

        esp_ble_gattc_get_char_by_uuid(gattc_if, p->conn_id,
                                      p->service_start_handle,
                                      p->service_end_handle,
                                      remote_filter_char_uuid,
                                      char_elem_result,
                                      &count);

        if (count > 0) {
            p->char_handle = char_elem_result[0].char_handle;
            ESP_LOGI(GATTC_TAG, "Stream char handle=0x%04x", p->char_handle);

            // Register for notifications
            esp_ble_gattc_register_for_notify(gattc_if, p->remote_bda, p->char_handle);
        } else {
            ESP_LOGE(GATTC_TAG, "Stream characteristic not found by UUID");
        }

        free(char_elem_result);
        char_elem_result = NULL;
        break;

    case ESP_GATTC_REG_FOR_NOTIFY_EVT: {
        if (param->reg_for_notify.status != ESP_GATT_OK) {
            ESP_LOGE(GATTC_TAG, "REG_FOR_NOTIFY failed, status=%d", param->reg_for_notify.status);
            break;
        }

        // Find CCCD descriptor
        uint16_t count = 0;
        esp_ble_gattc_get_attr_count(gattc_if, p->conn_id,
                                     ESP_GATT_DB_DESCRIPTOR,
                                     p->service_start_handle,
                                     p->service_end_handle,
                                     p->char_handle,
                                     &count);

        if (count == 0) {
            ESP_LOGE(GATTC_TAG, "No descriptors found");
            break;
        }

        descr_elem_result = (esp_gattc_descr_elem_t *)malloc(sizeof(esp_gattc_descr_elem_t) * count);
        if (!descr_elem_result) {
            ESP_LOGE(GATTC_TAG, "malloc descr_elem_result failed");
            break;
        }

        esp_ble_gattc_get_descr_by_char_handle(gattc_if,
                                              p->conn_id,
                                              p->char_handle,
                                              notify_descr_uuid,
                                              descr_elem_result,
                                              &count);

        if (count > 0) {
            p->cccd_handle = descr_elem_result[0].descr_handle;
            ESP_LOGI(GATTC_TAG, "CCCD handle=0x%04x (use buttons to enable/disable)", p->cccd_handle);
        } else {
            ESP_LOGE(GATTC_TAG, "CCCD descriptor not found");
        }

        free(descr_elem_result);
        descr_elem_result = NULL;
        break;
    }

    case ESP_GATTC_NOTIFY_EVT:
        if (param->notify.value_len == 16) {
            ESP_LOGI(GATTC_TAG, "Notify len=16");
            ESP_LOG_BUFFER_HEX(GATTC_TAG, param->notify.value, 16);
        } else {
            ESP_LOGW(GATTC_TAG, "Notify len=%d", param->notify.value_len);
            ESP_LOG_BUFFER_HEX(GATTC_TAG, param->notify.value, param->notify.value_len);
        }
        break;

    case ESP_GATTC_WRITE_DESCR_EVT:
        if (param->write.status != ESP_GATT_OK) {
            ESP_LOGE(GATTC_TAG, "WRITE_DESCR failed, status=0x%x", param->write.status);
            break;
        }
        // CCCD write completed
        if (param->write.handle == p->cccd_handle) {
            // We can't read back the value here without another read; keep local state in sync by checking last request:
            // If you want stronger sync, do esp_ble_gattc_read_char_descr() after write.
            // For now, infer from last "notify_enabled" toggle on request.
            // (We update state in write_cccd callers, but only after successful request; here confirms success.)
            ESP_LOGI(GATTC_TAG, "CCCD write confirmed");
        }
        break;

    case ESP_GATTC_DISCONNECT_EVT:
        ESP_LOGI(GATTC_TAG, "DISCONNECT_EVT, reason=0x%02x", param->disconnect.reason);
        connect_ok = false;
        get_server_ok = false;
        notify_enabled = false;
        p->cccd_handle = 0;
        p->char_handle = 0;
        esp_ble_gap_start_scanning(0);
        break;

    default:
        break;
    }
}

// --------------------- main ---------------------

void app_main(void)
{
    // Profile init
    memset(gl_profile_tab, 0, sizeof(gl_profile_tab));
    gl_profile_tab[PROFILE_A_APP_ID].gattc_cb = gattc_profile_event_handler;
    gl_profile_tab[PROFILE_A_APP_ID].gattc_if = ESP_GATT_IF_NONE;
    gl_profile_tab[PROFILE_A_APP_ID].app_id   = PROFILE_A_APP_ID;

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(GATTC_TAG, "bt_controller_init failed: %s", esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(GATTC_TAG, "bt_controller_enable failed: %s", esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(GATTC_TAG, "bluedroid_init failed: %s", esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(GATTC_TAG, "bluedroid_enable failed: %s", esp_err_to_name(ret));
        return;
    }

    ble_rf_tune_before_adv();

    // Register GAP callback
    ret = esp_ble_gap_register_callback(esp_gap_cb);
    if (ret) {
        ESP_LOGE(GATTC_TAG, "gap_register_callback failed: %s", esp_err_to_name(ret));
        return;
    }

    // Register GATTC callback
    ret = esp_ble_gattc_register_callback(esp_gattc_cb);
    if (ret) {
        ESP_LOGE(GATTC_TAG, "gattc_register_callback failed: %s", esp_err_to_name(ret));
        return;
    }

    // Register app/profile
    ret = esp_ble_gattc_app_register(PROFILE_A_APP_ID);
    if (ret) {
        ESP_LOGE(GATTC_TAG, "gattc_app_register failed: %s", esp_err_to_name(ret));
        return;
    }

    // Start button task (works even before connect; it will "skip" writes until CCCD discovered)
    xTaskCreate(button_task, "btn_task", 2048, NULL, 5, NULL);
}
