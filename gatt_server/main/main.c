/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
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

#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gatt_common_api.h"
#include "heart_rate.h"
#include "led.h"

#define PROFILE_NUM 2
#define HEART_PROFILE_APP_ID 0
#define AUTO_IO_PROFILE_APP_ID 1
#define HEART_RATE_SVC_UUID 0x180D
#define HEART_RATE_CHAR_UUID 0x2A37
#define HEART_NUM_HANDLE 4
#define AUTO_IO_SVC_UUID 0x1815
#define AUTO_IO_NUM_HANDLE 7

#define ADV_CONFIG_FLAG      (1 << 0)
#define SCAN_RSP_CONFIG_FLAG (1 << 1)

#define ESP_PWR_LVL ESP_PWR_LVL_P9

struct gatts_profile_inst {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
};

///Declare the static function
static void heart_gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
static void auto_io_gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
static void write_event_env_response(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

static const char *GATTS_TAG = "DPPS-DTS-TAG";
static esp_gatt_char_prop_t heart_property = 0;
static esp_gatt_char_prop_t auto_io_property = 0;
static uint8_t heart_rate_val[2] = {0};
static uint8_t led_status[2] = {0};
static bool indicate_enabled = false;
static bool hrs_create_cmpl = false;  // Heart Rate Service
static uint8_t adv_config_done = 0;

static esp_attr_value_t heart_rate_attr = {
    .attr_max_len = 2,
    .attr_len     = sizeof(heart_rate_val),
    .attr_value   = heart_rate_val,
};

static esp_attr_value_t led_status_attr = {
    .attr_max_len = 2,
    .attr_len     = sizeof(led_status),
    .attr_value   = led_status,
};

// ---- LED (Write) ----
// 128-bit UUID for the LED service (change if you want)
static const uint8_t led_chr_uuid[] = {
    0x23, 0xd1, 0xbc, 0xea, 0x5f, 0x78, 0x23, 0x15, 0xde, 0xef, 0x12, 0x12, 0x25, 0x15, 0x00, 0x00
};

// ---- STREAM NOTIFY (16 bytes every 10ms) ----
// 128-bit UUID for the streaming characteristic (change if you want)
static const uint8_t stream_chr_uuid[] = {
    0x23, 0xd1, 0xbc, 0xea, 0x5f, 0x78, 0x23, 0x15, 0xde, 0xef, 0x12, 0x12, 0x25, 0x15, 0x00, 0x01
};

static uint8_t stream_payload[16] = {0};

static esp_attr_value_t stream_attr = {
    .attr_max_len = sizeof(stream_payload),
    .attr_len     = sizeof(stream_payload),
    .attr_value   = stream_payload,
};

static bool stream_notify_enabled = false;
static uint16_t stream_char_handle = 0;
static uint16_t stream_cccd_handle = 0;
static esp_gatt_if_t stream_gatts_if = ESP_GATT_IF_NONE;
static uint16_t stream_conn_id = 0;
static TaskHandle_t stream_task_handle = NULL;
static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x0006,
    .max_interval = 0x0010,
    .appearance = 0x00,
    .manufacturer_len = 0,
    .p_manufacturer_data =  NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = 0,
    .p_service_uuid = NULL,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

static esp_ble_adv_params_t adv_params = {
    .adv_int_min        = 0x20,  // 20ms
    .adv_int_max        = 0x40,  // 40ms
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy  = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};
 
static struct gatts_profile_inst gl_profile_tab[PROFILE_NUM] = {
    [HEART_PROFILE_APP_ID] = {
        .gatts_cb = heart_gatts_profile_event_handler,
        .gatts_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
    [AUTO_IO_PROFILE_APP_ID] = {
        .gatts_cb = auto_io_gatts_profile_event_handler,
        .gatts_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
};

static void heart_rate_task(void* param)
{
    ESP_LOGI(GATTS_TAG, "Heart Rate Task Start");

    while (1) {
        if (hrs_create_cmpl) {
            update_heart_rate();
            ESP_LOGI(GATTS_TAG, "Heart Rate updated to %d", get_heart_rate());

            heart_rate_val[0] = 0;
            heart_rate_val[1] = get_heart_rate();
            esp_ble_gatts_set_attr_value(gl_profile_tab[HEART_PROFILE_APP_ID].char_handle, 2, heart_rate_val);
        }

        vTaskDelay(pdMS_TO_TICKS(1000));

        // vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

static void stream_notify_task(void *arg)
{
    TickType_t last = xTaskGetTickCount();
    uint32_t counter = 0;
    ESP_LOGI(GATTS_TAG, "Stream notify Task Start");

    while (1) {

        if (stream_notify_enabled && stream_gatts_if != ESP_GATT_IF_NONE && stream_char_handle != 0) {

          	// Example payload: counter in first 4 bytes + pattern
            memcpy(stream_payload, &counter, sizeof(counter));
            for (int i = 4; i < 16; i++) {
                stream_payload[i] = (uint8_t)(counter + i);
            }

            // need_confirm=false => NOTIFY
            esp_err_t err = esp_ble_gatts_send_indicate(
                stream_gatts_if,
                stream_conn_id,
                stream_char_handle,
                sizeof(stream_payload),
                stream_payload,
                false
            );
            if (err != ESP_OK) {
                ESP_LOGW(GATTS_TAG, "stream notify failed: %s", esp_err_to_name(err));
            }
            counter++;
        }

        vTaskDelayUntil(&last, pdMS_TO_TICKS(10));
    }
}

static void ble_rf_tune_before_adv(void)
{
    // 1) TX power al massimo per BLE (ADV/SCAN/DEFAULT)
    // Nota: i livelli disponibili dipendono dal chip/target; in genere P9 è il più alto.
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, ESP_PWR_LVL);
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV,     ESP_PWR_LVL);
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_SCAN,    ESP_PWR_LVL);


    // 2) (Opzionale) Preferisci PHY più robusto in connessione.
    // Se vuoi massimo range/stabilità: S8 coded. Se vuoi compromesso: S2 coded.
    // Nota: avrà effetto solo se anche il peer supporta LE Coded PHY.
    esp_ble_gap_set_preferred_default_phy(
        ESP_BLE_GAP_PHY_CODED_PREF_MASK | ESP_BLE_GAP_PHY_1M_PREF_MASK,
        ESP_BLE_GAP_PHY_CODED_PREF_MASK | ESP_BLE_GAP_PHY_1M_PREF_MASK
    );
}

static void ble_rf_tune_on_connect(esp_bd_addr_t peer_bda)
{
    // TX power massima anche sul link di connessione (handle 0)
    // Se hai più connessioni, userai CONN_HDL1, CONN_HDL2, ...
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_CONN_HDL0, ESP_PWR_LVL);

        // all_phys_mask = 0 -> dichiari che stai fornendo preferenze TX/RX
    // tx/rx mask: preferisci CODED (puoi anche aggiungere 1M come fallback)
    esp_err_t err = esp_ble_gap_set_preferred_phy(
        peer_bda,
        0,
        ESP_BLE_GAP_PHY_CODED_PREF_MASK | ESP_BLE_GAP_PHY_1M_PREF_MASK,
        ESP_BLE_GAP_PHY_CODED_PREF_MASK | ESP_BLE_GAP_PHY_1M_PREF_MASK,
        ESP_BLE_GAP_PHY_OPTIONS_PREF_S8_CODING
    );

    if (err != ESP_OK) {
        ESP_LOGW("BLE", "set_preferred_phy failed: %s", esp_err_to_name(err));
    }
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        ESP_LOGI(GATTS_TAG, "Advertising data set, status %d", param->adv_data_cmpl.status);
        adv_config_done &= (~ADV_CONFIG_FLAG);
        if (adv_config_done == 0) {
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
    case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
        ESP_LOGI(GATTS_TAG, "Scan response data set, status %d", param->scan_rsp_data_cmpl.status);
        adv_config_done &= (~SCAN_RSP_CONFIG_FLAG);
        if (adv_config_done == 0) {
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(GATTS_TAG, "Advertising start failed, status %d", param->adv_start_cmpl.status);
            break;
        }
        ESP_LOGI(GATTS_TAG, "Advertising start successfully");
        break;
    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
        ESP_LOGI(GATTS_TAG, "Connection params update, status %d, conn_int %d, latency %d, timeout %d",
                 param->update_conn_params.status,
                 param->update_conn_params.conn_int,
                 param->update_conn_params.latency,
                 param->update_conn_params.timeout);
        break;
    case ESP_GAP_BLE_SET_PKT_LENGTH_COMPLETE_EVT:
        ESP_LOGI(GATTS_TAG, "Packet length update, status %d, rx %d, tx %d",
                 param->pkt_data_length_cmpl.status,
                 param->pkt_data_length_cmpl.params.rx_len,
                 param->pkt_data_length_cmpl.params.tx_len);
        break;
    case ESP_GAP_BLE_PHY_UPDATE_COMPLETE_EVT:
        ESP_LOGI(GATTS_TAG, "PHY update: TX=%d RX=%d",
            param->phy_update.tx_phy,
            param->phy_update.rx_phy
        );
        break;
    default:
        break;
    }
}

static void heart_gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    switch (event) {
    case ESP_GATTS_REG_EVT:
        ESP_LOGI(GATTS_TAG, "GATT server register, status %d, app_id %d", param->reg.status, param->reg.app_id);
        gl_profile_tab[HEART_PROFILE_APP_ID].service_id.is_primary = true;
        gl_profile_tab[HEART_PROFILE_APP_ID].service_id.id.inst_id = 0x00;
        gl_profile_tab[HEART_PROFILE_APP_ID].service_id.id.uuid.len = ESP_UUID_LEN_16;
        gl_profile_tab[HEART_PROFILE_APP_ID].service_id.id.uuid.uuid.uuid16 = HEART_RATE_SVC_UUID;

        //config adv data
        esp_err_t ret = esp_ble_gap_config_adv_data(&adv_data);
        if (ret) {
            ESP_LOGE(GATTS_TAG, "config adv data failed, error code = %x", ret);
            break;
        }

        esp_ble_gatts_create_service(gatts_if, &gl_profile_tab[HEART_PROFILE_APP_ID].service_id, HEART_NUM_HANDLE);
        break;
    case ESP_GATTS_CREATE_EVT:
        //service has been created, now add characteristic declaration
        ESP_LOGI(GATTS_TAG, "Service create, status %d, service_handle %d", param->create.status, param->create.service_handle);
        gl_profile_tab[HEART_PROFILE_APP_ID].service_handle = param->create.service_handle;
        gl_profile_tab[HEART_PROFILE_APP_ID].char_uuid.len = ESP_UUID_LEN_16;
        gl_profile_tab[HEART_PROFILE_APP_ID].char_uuid.uuid.uuid16 = HEART_RATE_CHAR_UUID;
        esp_ble_gatts_start_service(gl_profile_tab[HEART_PROFILE_APP_ID].service_handle);
        heart_property = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_INDICATE;
        ret = esp_ble_gatts_add_char(gl_profile_tab[HEART_PROFILE_APP_ID].service_handle, &gl_profile_tab[HEART_PROFILE_APP_ID].char_uuid,
                            ESP_GATT_PERM_READ,
                            heart_property,
                            &heart_rate_attr, NULL);
        if (ret) {
            ESP_LOGE(GATTS_TAG, "add char failed, error code = %x", ret);
        }
        break;
    case ESP_GATTS_READ_EVT:
        ESP_LOGI(GATTS_TAG, "Characteristic read");
        esp_gatt_rsp_t rsp;
        memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
        rsp.attr_value.handle = param->read.handle;
        rsp.attr_value.len = 2;
        memcpy(rsp.attr_value.value, heart_rate_val, sizeof(heart_rate_val));
        esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id, ESP_GATT_OK, &rsp);
        break;
    
    case ESP_GATTS_WRITE_EVT:

        ESP_LOGI(GATTS_TAG, "ESP_GATTS_WRITE_EVT: executed");

        write_event_env_response(gatts_if, param);
        break;

    case ESP_GATTS_DELETE_EVT:
        break;
    case ESP_GATTS_START_EVT:
        ESP_LOGI(GATTS_TAG, "Service start, status %d, service_handle %d", param->start.status, param->start.service_handle);
        break;
    case ESP_GATTS_STOP_EVT:
        break;
    case ESP_GATTS_CONNECT_EVT:
        ESP_LOGI(GATTS_TAG, "Connected, conn_id %u, remote "ESP_BD_ADDR_STR"",
                param->connect.conn_id, ESP_BD_ADDR_HEX(param->connect.remote_bda));
        gl_profile_tab[HEART_PROFILE_APP_ID].conn_id = param->connect.conn_id;

        ble_rf_tune_on_connect(param->connect.remote_bda);

        break;

    case ESP_GATTS_DISCONNECT_EVT:
            ESP_LOGI(GATTS_TAG, "HEART disconnected, remote "ESP_BD_ADDR_STR", reason 0x%02x",
                    ESP_BD_ADDR_HEX(param->disconnect.remote_bda), param->disconnect.reason);
            esp_ble_gap_start_advertising(&adv_params);
        break;
    case ESP_GATTS_CONF_EVT:
        ESP_LOGI(GATTS_TAG, "Confirm receive, status %d, attr_handle %d", param->conf.status, param->conf.handle);
        if (param->conf.status != ESP_GATT_OK) {
            ESP_LOG_BUFFER_HEX(GATTS_TAG, param->conf.value, param->conf.len);
        }
        break;
    case ESP_GATTS_SET_ATTR_VAL_EVT:
        ESP_LOGI(GATTS_TAG, "Attribute value set, status %d", param->set_attr_val.status);
        if (indicate_enabled) {
            uint8_t indicate_data[2] = {0};
            memcpy(indicate_data, heart_rate_val, sizeof(heart_rate_val));
            esp_ble_gatts_send_indicate(gatts_if, gl_profile_tab[HEART_PROFILE_APP_ID].conn_id, gl_profile_tab[HEART_PROFILE_APP_ID].char_handle, sizeof(indicate_data), indicate_data, true);
        }
        break;
    default:
        break;
    }
}

static void auto_io_gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    switch (event) {
    case ESP_GATTS_REG_EVT:
        ESP_LOGI(GATTS_TAG, "GATT server register, status %d, app_id %d", param->reg.status, param->reg.app_id);
        gl_profile_tab[AUTO_IO_PROFILE_APP_ID].service_id.is_primary = true;
        gl_profile_tab[AUTO_IO_PROFILE_APP_ID].service_id.id.inst_id = 0x00;
        gl_profile_tab[AUTO_IO_PROFILE_APP_ID].service_id.id.uuid.len = ESP_UUID_LEN_16;
        gl_profile_tab[AUTO_IO_PROFILE_APP_ID].service_id.id.uuid.uuid.uuid16 = AUTO_IO_SVC_UUID;
        esp_ble_gatts_create_service(gatts_if, &gl_profile_tab[AUTO_IO_PROFILE_APP_ID].service_id, AUTO_IO_NUM_HANDLE);
        break;
    case ESP_GATTS_CREATE_EVT:
        //service has been created, now add characteristic declaration
        ESP_LOGI(GATTS_TAG, "Service create, status %d, service_handle %d", param->create.status, param->create.service_handle);
        gl_profile_tab[AUTO_IO_PROFILE_APP_ID].service_handle = param->create.service_handle;
        gl_profile_tab[AUTO_IO_PROFILE_APP_ID].char_uuid.len = ESP_UUID_LEN_128;
        memcpy(gl_profile_tab[AUTO_IO_PROFILE_APP_ID].char_uuid.uuid.uuid128, led_chr_uuid, ESP_UUID_LEN_128);

        esp_ble_gatts_start_service(gl_profile_tab[AUTO_IO_PROFILE_APP_ID].service_handle);
        auto_io_property = ESP_GATT_CHAR_PROP_BIT_WRITE ;
        esp_err_t ret = esp_ble_gatts_add_char(gl_profile_tab[AUTO_IO_PROFILE_APP_ID].service_handle, &gl_profile_tab[AUTO_IO_PROFILE_APP_ID].char_uuid,
                            ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE ,
                            auto_io_property,
                            &led_status_attr, NULL);
        if (ret) {
            ESP_LOGE(GATTS_TAG, "add char failed, error code = %x", ret);
        }
		
		// Add STREAM characteristic (NOTIFY, 16 bytes)
		esp_bt_uuid_t stream_uuid = { .len = ESP_UUID_LEN_128 };
		memcpy(stream_uuid.uuid.uuid128, stream_chr_uuid, ESP_UUID_LEN_128);
		
		esp_gatt_char_prop_t stream_prop = ESP_GATT_CHAR_PROP_BIT_NOTIFY;
		ret = esp_ble_gatts_add_char(gl_profile_tab[AUTO_IO_PROFILE_APP_ID].service_handle,
		                             &stream_uuid,
		                             ESP_GATT_PERM_READ,
		                             stream_prop,
		                             &stream_attr,
		                             NULL);
		if (ret) {
		    ESP_LOGE(GATTS_TAG, "add STREAM char failed, error code = %x", ret);
		}
        break;

    case ESP_GATTS_ADD_CHAR_EVT:
        ESP_LOGI(GATTS_TAG, "AUTO_IO: add char, status %d, attr_handle %d",
                param->add_char.status, param->add_char.attr_handle);

        // Distinguish LED characteristic vs STREAM characteristic by UUID
        if (param->add_char.char_uuid.len == ESP_UUID_LEN_128 &&
            memcmp(param->add_char.char_uuid.uuid.uuid128, stream_chr_uuid, ESP_UUID_LEN_128) == 0) {

            stream_char_handle = param->add_char.attr_handle;
            ESP_LOGI(GATTS_TAG, "STREAM char handle=%u", stream_char_handle);

            // Add CCCD for notifications
            esp_bt_uuid_t cccd_uuid = {
                .len = ESP_UUID_LEN_16,
                .uuid = { .uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG },
            };

            esp_err_t dret = esp_ble_gatts_add_char_descr(
                gl_profile_tab[AUTO_IO_PROFILE_APP_ID].service_handle,
                &cccd_uuid,
                ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                NULL,
                NULL
            );
            if (dret) {
                ESP_LOGE(GATTS_TAG, "add STREAM CCCD failed, error code = %x", dret);
            }
        } else {
            // LED characteristic handle
            gl_profile_tab[AUTO_IO_PROFILE_APP_ID].char_handle = param->add_char.attr_handle;
            ESP_LOGI(GATTS_TAG, "LED char handle=%u", gl_profile_tab[AUTO_IO_PROFILE_APP_ID].char_handle);
        }
    break;

    case ESP_GATTS_ADD_CHAR_DESCR_EVT:
        ESP_LOGI(GATTS_TAG, "AUTO_IO: add descr, status %d, handle %d",
                param->add_char_descr.status, param->add_char_descr.attr_handle);

        // This example only adds one descriptor (STREAM CCCD)
        stream_cccd_handle = param->add_char_descr.attr_handle;
        ESP_LOGI(GATTS_TAG, "STREAM CCCD handle=%u", stream_cccd_handle);
    break;

    case ESP_GATTS_READ_EVT:
        ESP_LOGI(GATTS_TAG, "Characteristic read");
        esp_gatt_rsp_t rsp;
        memset(&rsp, 0, sizeof(esp_gatt_rsp_t));

        rsp.attr_value.handle = param->read.handle;
        rsp.attr_value.len = 1;
        rsp.attr_value.value[0] = 0x02;
        esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id, ESP_GATT_OK, &rsp);
        break;

    case ESP_GATTS_WRITE_EVT:
        ESP_LOGI(GATTS_TAG, "AUTO_IO: write, handle=%u, len=%u", param->write.handle, param->write.len);
        ESP_LOG_BUFFER_HEX(GATTS_TAG, param->write.value, param->write.len);

        // STREAM CCCD write (enable/disable notifications)
        if (!param->write.is_prep &&
            param->write.handle == stream_cccd_handle &&
            param->write.len == 2) {

            uint16_t cccd = (param->write.value[1] << 8) | param->write.value[0];
            stream_notify_enabled = (cccd & 0x0001);

            ESP_LOGI(GATTS_TAG, "STREAM notify %s", stream_notify_enabled ? "ENABLED" : "DISABLED");

            if (stream_notify_enabled) {
                // Start task if not running
                if (stream_task_handle == NULL) {
                    xTaskCreate(stream_notify_task, "Stream Notify", 3 * 1024, NULL, 5, &stream_task_handle);
                }
            } else {
                // Stop task if running
                if (stream_task_handle != NULL) {
                    vTaskDelete(stream_task_handle);
                    stream_task_handle = NULL;
                }
            }
        }

        // LED characteristic write (existing behavior)
        if (!param->write.is_prep &&
            param->write.handle == gl_profile_tab[AUTO_IO_PROFILE_APP_ID].char_handle &&
            param->write.len >= 1) {

            if (param->write.value[0]) {
                ESP_LOGI(GATTS_TAG, "LED ON!");
                led_on();
            } else {
                ESP_LOGI(GATTS_TAG, "LED OFF!");
                led_off();
            }
        }

        write_event_env_response(gatts_if, param);
        break;

    case ESP_GATTS_DELETE_EVT:
        break;
    case ESP_GATTS_START_EVT:
        ESP_LOGI(GATTS_TAG, "Service start, status %d, service_handle %d", param->start.status, param->start.service_handle);
        break;
    case ESP_GATTS_STOP_EVT:
        break;
    case ESP_GATTS_CONNECT_EVT:
        esp_ble_conn_update_params_t conn_params = {0};
        memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
        conn_params.latency = 0;
        conn_params.max_int = 0x10; // 20ms
        conn_params.min_int = 0x08; // 10ms
        conn_params.timeout = 400;
        ESP_LOGI(GATTS_TAG, "Connected, conn_id %u, remote "ESP_BD_ADDR_STR"",
                param->connect.conn_id, ESP_BD_ADDR_HEX(param->connect.remote_bda));
        gl_profile_tab[AUTO_IO_PROFILE_APP_ID].conn_id = param->connect.conn_id;

        
        stream_gatts_if = gatts_if;
        stream_conn_id = param->connect.conn_id;
		ble_rf_tune_on_connect(param->connect.remote_bda);
        
        esp_ble_gap_update_conn_params(&conn_params);
        break;

    case ESP_GATTS_DISCONNECT_EVT:
        ESP_LOGI(GATTS_TAG, "AUTO_IO disconnected, remote "ESP_BD_ADDR_STR", reason 0x%02x",
                ESP_BD_ADDR_HEX(param->disconnect.remote_bda), param->disconnect.reason);

        stream_notify_enabled = false;
        stream_gatts_if = ESP_GATT_IF_NONE;
        stream_conn_id = 0;

        if (stream_task_handle != NULL) {
            vTaskDelete(stream_task_handle);
            stream_task_handle = NULL;
        }

        esp_ble_gap_start_advertising(&adv_params);
    break;

    case ESP_GATTS_CONF_EVT:
        ESP_LOGI(GATTS_TAG, "Confirm receive, status %d, attr_handle %d", param->conf.status, param->conf.handle);
        if (param->conf.status != ESP_GATT_OK) {
            ESP_LOG_BUFFER_HEX(GATTS_TAG, param->conf.value, param->conf.len);
        }
        break;
    default:
        break;
    }
}

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    if (event == ESP_GATTS_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            gl_profile_tab[param->reg.app_id].gatts_if = gatts_if;

        } else {
            ESP_LOGI(GATTS_TAG, "Reg app failed, app_id %04x, status %d",
                    param->reg.app_id,
                    param->reg.status);
            return;
        }
    }

    //gatts_if registered complete, call cb handlers
    do {
        int idx;
        for (idx = 0; idx < PROFILE_NUM; idx++) {
            if (gatts_if == ESP_GATT_IF_NONE || /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
                    gatts_if == gl_profile_tab[idx].gatts_if) {
                if (gl_profile_tab[idx].gatts_cb) {
                    gl_profile_tab[idx].gatts_cb(event, gatts_if, param);
                }
            }
        }
    } while(0);
}


void app_main(void)
{
    esp_err_t ret;

    led_init();

    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s initialize controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s init bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s enable bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ble_rf_tune_before_adv();

    ESP_ERROR_CHECK(esp_ble_gap_set_device_name("DPPS-DTS"));

    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret) {
        ESP_LOGE(GATTS_TAG, "gap register error, error code = %x", ret);
        return;
    }

    ret = esp_ble_gatts_register_callback(gatts_event_handler);
    if (ret) {
        ESP_LOGE(GATTS_TAG, "gatts register error, error code = %x", ret);
        return;
    }

    ret = esp_ble_gatts_app_register(HEART_PROFILE_APP_ID);
    if (ret) {
        ESP_LOGE(GATTS_TAG, "app register error, error code = %x", ret);
        return;
    }

    ret = esp_ble_gatts_app_register(AUTO_IO_PROFILE_APP_ID);
    if (ret) {
        ESP_LOGE(GATTS_TAG, "app register error, error code = %x", ret);
        return;
    }

    ret = esp_ble_gatt_set_local_mtu(500);
    if (ret) {
        ESP_LOGE(GATTS_TAG, "set local  MTU failed, error code = %x", ret);
    }

    xTaskCreate(heart_rate_task, "Heart Rate", 2 * 1024, NULL, 5, NULL);
}

void write_event_env_response(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    esp_gatt_status_t status = ESP_GATT_OK;
    if (param->write.need_rsp) {
        esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, NULL);
    }
}
