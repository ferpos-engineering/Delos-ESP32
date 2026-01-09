/*
 * main.c - ESP-IDF 5.5.1 (Bluedroid) GATT Client
 *
 * Modifiche richieste:
 *  - usa 2 pulsanti GPIO per abilitare/disabilitare NOTIFY scrivendo 0x0001/0x0000 nella CCCD (0x2902)
 *  - cerca un server con nome REMOTE_DEVICE_NAME
 *  - trova servizio 16-bit REMOTE_SERVICE_UUID (AUTO_IO = 0x1815) e characteristic STREAM (UUID 128-bit ...0001)
 *  - Versione “PRO”: statistiche NOTIFY (min/max/avg/jitter) stampate 1 volta al secondo
 *
 * NOTE:
 *  - Aggiorna BTN_ENABLE_GPIO / BTN_DISABLE_GPIO in base alla tua board.
 *  - In main/CMakeLists.txt aggiungi: REQUIRES driver bt nvs_flash
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <limits.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_system.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"

#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_gatt_defs.h"
#include "esp_gatt_common_api.h"

#include "driver/gpio.h"

#include "dataloss.h"
#include "prettyprinter.h"

#ifndef INVALID_HANDLE
#define INVALID_HANDLE 0
#endif

// --------------------- CONFIG ---------------------

#define MTU                         23
#define PROFILE_NUM                 1
#define PROFILE_A_APP_ID            0

#define REMOTE_DEVICE_NAME          "DPPS-DTS"
#define REMOTE_SERVICE_UUID         0x1815   // AUTO_IO_SVC_UUID nel tuo server
#define LED_DATALOSS                 6
#define LED_THROTTLING_MS            2000

// Buttons (ACTIVE LOW)
#if CONFIG_IDF_TARGET_ESP32C6
#define BTN_ENABLE_GPIO             9
#define BTN_DISABLE_GPIO            8
#else
#define BTN_ENABLE_GPIO             0
#define BTN_DISABLE_GPIO            2
#endif

#define BTN_POLL_MS                 20
#define BTN_DEBOUNCE_MS             50

#define ESP_PWR_LVL ESP_PWR_LVL_N0

static const char *DEVICE_NAME = "DPPS-DONGLE";

// --------------------- Scan params ---------------------

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
    uint16_t led_char_handle;   // Handle della characteristic LED (WRITE) per comandare lo stato sul server
    esp_bd_addr_t remote_bda;
};

// STREAM characteristic UUID (128-bit) ...0001 (deve combaciare col server)
static esp_bt_uuid_t remote_filter_char_uuid = {
    .len = ESP_UUID_LEN_128,
    .uuid = {.uuid128 = {
        0x23, 0xd1, 0xbc, 0xea, 0x5f, 0x78, 0x23, 0x15,
        0xde, 0xef, 0x12, 0x12, 0x25, 0x15, 0x00, 0x01
    }},
};

// LED characteristic UUID (128-bit) ...0000 (deve combaciare col server) - usata per WRITE dello stato LED
static esp_bt_uuid_t remote_led_char_uuid = {
    .len = ESP_UUID_LEN_128,
    .uuid = {.uuid128 = {
        0x23, 0xd1, 0xbc, 0xea, 0x5f, 0x78, 0x23, 0x15,
        0xde, 0xef, 0x12, 0x12, 0x25, 0x15, 0x00, 0x00
    }},
};


// CCCD UUID 0x2902
static esp_bt_uuid_t notify_descr_uuid = {
    .len = ESP_UUID_LEN_16,
    .uuid = {.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG,},
};

static struct gattc_profile_inst gl_profile_tab[PROFILE_NUM];

static bool connect_ok    = false;
static bool service_ok    = false;

static esp_gattc_char_elem_t  *char_elem_result  = NULL;
static esp_gattc_descr_elem_t *descr_elem_result = NULL;

// --------------------- NOTIFY stats (PRO) ---------------------

typedef struct {
    int64_t last_us;

    uint32_t count;
    int64_t sum_us;
    int64_t min_us;
    int64_t max_us;

    int64_t window_start_us;
} notify_stats_t;

static notify_stats_t notify_stats = {
    .last_us = -1,
    .count = 0,
    .sum_us = 0,
    .min_us = INT64_MAX,
    .max_us = 0,
    .window_start_us = 0,
};

// throttling write LED_DATALOSS verso server
static int64_t s_last_led_dataloss_cmd_us = -1;

// --------------------- STREAM duration ---------------------

typedef struct {
    bool     active;
    int64_t  start_us;   // timestamp del primo NOTIFY ricevuto
    int64_t  last_us;    // timestamp dell'ultimo NOTIFY ricevuto
    uint32_t packets;
    uint64_t bytes;
} stream_timer_t;

static stream_timer_t stream_t = {
    .active = false,
    .start_us = -1,
    .last_us = -1,
    .packets = 0,
    .bytes = 0,
};


// --------------------- Helpers ---------------------

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

static bool adv_name_matches(const uint8_t *adv_data, const char *name)
{
    uint8_t len = 0;
    // API vuole uint8_t* (non const), facciamo cast (adv_data non viene modificato)
    uint8_t *p = esp_ble_resolve_adv_data((uint8_t *)adv_data, ESP_BLE_AD_TYPE_NAME_CMPL, &len);
    if (p && len) {
        size_t nlen = strlen(name);
        return (nlen == len) && (memcmp(p, name, len) == 0);
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

static esp_err_t cccd_write(uint16_t value)
{
    struct gattc_profile_inst *p = &gl_profile_tab[PROFILE_A_APP_ID];

    if (!connect_ok || !service_ok || p->cccd_handle == 0 || p->gattc_if == ESP_GATT_IF_NONE) {
        ESP_LOGW(DEVICE_NAME, "CCCD write skipped (connect_ok=%d service_ok=%d cccd=0x%04x)",
                 connect_ok, service_ok, p->cccd_handle);
        return ESP_FAIL;
    }

    // CCCD è 2 byte little-endian
    uint8_t cccd[2] = { (uint8_t)(value & 0xFF), (uint8_t)((value >> 8) & 0xFF) };

    esp_err_t err = esp_ble_gattc_write_char_descr(
        p->gattc_if,
        p->conn_id,
        p->cccd_handle,
        sizeof(cccd),
        cccd,
        ESP_GATT_WRITE_TYPE_RSP,
        ESP_GATT_AUTH_REQ_NONE
    );

    if (err != ESP_OK) {
        ESP_LOGE(DEVICE_NAME, "write_char_descr failed: %s", esp_err_to_name(err));
        return err;
    }

    dataloss_reset();
    ESP_LOGI(DEVICE_NAME, "CCCD write requested: 0x%04x", value);

    // STREAM duration: arma/reset su enable, stampa su disable
    if (value == 0x0001) {
        stream_t.active = true;
        stream_t.start_us = -1;
        stream_t.last_us  = -1;
        stream_t.packets  = 0;
        stream_t.bytes    = 0;
    } else if (value == 0x0000) {
        if (stream_t.active && stream_t.start_us >= 0 && stream_t.last_us >= 0) {
            int64_t dur_us = stream_t.last_us - stream_t.start_us;
            ESP_LOGI(DEVICE_NAME,
                     "STREAM duration: %.3f s, packets=%" PRIu32 ", bytes=%" PRIu64,
                     dur_us / 1000000.0, stream_t.packets, stream_t.bytes);
        } else {
            ESP_LOGW(DEVICE_NAME, "STREAM duration: no data received");
        }
        stream_t.active = false;
    }

    return ESP_OK;
}

/**
 * @brief Scrive sul server lo stato LED (1 byte) tramite la characteristic LED (UUID ...0000).
 *
 * Il server interpreta il primo byte scritto come "state_num" e chiama led_on(state_num).
 * Esempio: LED_DATALOSS=6 -> scrivere 0x06.
 *
 * Questa funzione è "best effort": se non siamo connessi o non abbiamo ancora scoperto
 * l'handle della characteristic LED, non fa nulla e logga un warning.
 */
static esp_err_t led_write_state(uint8_t state_num)
{
    struct gattc_profile_inst *p = &gl_profile_tab[PROFILE_A_APP_ID];
    if (!connect_ok || !service_ok || p->gattc_if == ESP_GATT_IF_NONE || p->conn_id == 0xFFFF || p->led_char_handle == 0) {
        ESP_LOGW(DEVICE_NAME, "LED write skipped (conn/service/handle not ready)");
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t v = state_num;
    // WRITE con risposta (più robusto); il server ha permessi READ|WRITE sulla char LED
    esp_err_t err = esp_ble_gattc_write_char(
        p->gattc_if,
        p->conn_id,
        p->led_char_handle,
        sizeof(v),
        &v,
        ESP_GATT_WRITE_TYPE_RSP,
        ESP_GATT_AUTH_REQ_NONE
    );
    if (err != ESP_OK) {
        ESP_LOGW(DEVICE_NAME, "LED write failed: %s", esp_err_to_name(err));
    }
    return err;
}


static void button_task(void *arg)
{
    buttons_init();

    int prev_en  = 1;
    int prev_dis = 1;

    while (1) {
        int en  = gpio_get_level(BTN_ENABLE_GPIO);
        int dis = gpio_get_level(BTN_DISABLE_GPIO);

        if (prev_en == 1 && en == 0) {
            vTaskDelay(pdMS_TO_TICKS(BTN_DEBOUNCE_MS));
            if (gpio_get_level(BTN_ENABLE_GPIO) == 0) {
                ESP_LOGI(DEVICE_NAME, "BTN_ENABLE -> enable notify");
                cccd_write(0x0001);
            }
        }

        if (prev_dis == 1 && dis == 0) {
            vTaskDelay(pdMS_TO_TICKS(BTN_DEBOUNCE_MS));
            if (gpio_get_level(BTN_DISABLE_GPIO) == 0) {
                ESP_LOGI(DEVICE_NAME, "BTN_DISABLE -> disable notify");
                cccd_write(0x0000);
            }
        }

        prev_en  = en;
        prev_dis = dis;
        vTaskDelay(pdMS_TO_TICKS(BTN_POLL_MS));
    }
}

// --------------------- Callbacks ---------------------

static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
static void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);
static void gattc_profile_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);

static void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
    if (event == ESP_GATTC_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            gl_profile_tab[param->reg.app_id].gattc_if = gattc_if;
        } else {
            ESP_LOGE(DEVICE_NAME, "REG_EVT failed, status=%d", param->reg.status);
            return;
        }
    }

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
    switch (event)
    {
        case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT:
            esp_ble_gap_start_scanning(0);
            break;

        case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
            ESP_LOGI(DEVICE_NAME, "Scan start status=%d", param->scan_start_cmpl.status);
            break;

        case ESP_GAP_BLE_SCAN_RESULT_EVT:
        {
            switch (param->scan_rst.search_evt)
            {
                case ESP_GAP_SEARCH_INQ_RES_EVT:
                    if (adv_name_matches(param->scan_rst.ble_adv, REMOTE_DEVICE_NAME)) {
                        ESP_LOGI(DEVICE_NAME, "Found '%s' -> connect", REMOTE_DEVICE_NAME);
                        esp_ble_gap_stop_scanning();
                        esp_ble_gattc_open(gl_profile_tab[PROFILE_A_APP_ID].gattc_if,
                                        param->scan_rst.bda,
                                        param->scan_rst.ble_addr_type,
                                        true);
                    }
                    break;

                case ESP_GAP_SEARCH_INQ_CMPL_EVT:
                    ESP_LOGI(DEVICE_NAME, "Scan complete");
                    break;

                default:
                    break;
            }

            break;
        }

        case ESP_GAP_BLE_READ_RSSI_COMPLETE_EVT:
        {
            esp_ble_gap_cb_param_t *rssi = param;

            if (rssi->read_rssi_cmpl.status == ESP_BT_STATUS_SUCCESS)
            {
                ESP_LOGI(DEVICE_NAME,
                        "RSSI read complete: %d dBm (addr=%02X:%02X:%02X:%02X:%02X:%02X)",
                        rssi->read_rssi_cmpl.rssi,
                        rssi->read_rssi_cmpl.remote_addr[0],
                        rssi->read_rssi_cmpl.remote_addr[1],
                        rssi->read_rssi_cmpl.remote_addr[2],
                        rssi->read_rssi_cmpl.remote_addr[3],
                        rssi->read_rssi_cmpl.remote_addr[4],
                        rssi->read_rssi_cmpl.remote_addr[5]);
            }
            else
            {
                ESP_LOGW(DEVICE_NAME,
                        "RSSI read failed, status=%d",
                        rssi->read_rssi_cmpl.status);
            }
            break;
        }

        case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
        {
            const esp_ble_gap_cb_param_t *u = param;
            ESP_LOGI(DEVICE_NAME,
                "CONN_PARAMS status=%d, min_int=%d max_int=%d latency=%d timeout=%d",
                u->update_conn_params.status,
                u->update_conn_params.min_int,
                u->update_conn_params.max_int,
                u->update_conn_params.latency,
                u->update_conn_params.timeout
            );

            // conversioni comode
            float min_ms = u->update_conn_params.min_int * 1.25f;
            float max_ms = u->update_conn_params.max_int * 1.25f;
            float sup_ms = u->update_conn_params.timeout * 10.0f;
            ESP_LOGI(DEVICE_NAME, " -> interval=[%.2f..%.2f] ms, supervision=%.0f ms",
                    min_ms, max_ms, sup_ms);
            break;
        }

        case ESP_GAP_BLE_READ_PHY_COMPLETE_EVT:
        {
            char tx[6], rx[6];
            prettyprinter_print_phy(param->read_phy.tx_phy, tx, sizeof(tx));
            prettyprinter_print_phy(param->read_phy.rx_phy, rx, sizeof(rx));

            ESP_LOGI(DEVICE_NAME, "READ_PHY status=%d, tx_phy=%s rx_phy=%s",
                    param->read_phy.status, tx, rx);
            break;
        }

        case ESP_GAP_BLE_PHY_UPDATE_COMPLETE_EVT:
        {
            char tx[6], rx[6];
            prettyprinter_print_phy(param->phy_update.tx_phy, tx, sizeof(tx));
            prettyprinter_print_phy(param->phy_update.rx_phy, rx, sizeof(rx));
            ESP_LOGI(DEVICE_NAME, "PHY_UPDATE status=%d, tx_phy=%s rx_phy=%s",
                    param->phy_update.status, tx, rx);
            break;
        }

        case ESP_GAP_BLE_SET_PKT_LENGTH_COMPLETE_EVT:
            ESP_LOGI(DEVICE_NAME, "PKT_LEN status=%d, rx_len=%d tx_len=%d",
                    param->pkt_data_length_cmpl.status,
                    param->pkt_data_length_cmpl.params.rx_len,
                    param->pkt_data_length_cmpl.params.tx_len);
            break;

        case ESP_GAP_BLE_READ_REMOTE_TRANS_PWR_LEVEL_EVT:
        {
            ESP_LOGI(DEVICE_NAME, "READ_REMOTE_TRANS_PWR_LEVEL complete: status=%d",
                    param->read_remote_trans_pwr_level_cmpl.status);
            break;
        }

        case ESP_GAP_BLE_TRANS_PWR_RPTING_EVT:
        {
            const int8_t pwr = param->trans_power_report_evt.tx_power_level;

            char phy[10];
            prettyprinter_print_phy2(param->trans_power_report_evt.phy, phy, sizeof(phy));

            // Nota: 0x7E e 0x7F sono valori "speciali" (not managing / not available)
            if ((uint8_t)pwr == 0x7E) {
                ESP_LOGW(DEVICE_NAME, "Remote TX POWER report: remote not managing power on this PHY (conn=0x%04X phy=%s reason=%u)",
                        param->trans_power_report_evt.conn_handle,
                        phy,
                        param->trans_power_report_evt.reason);
            } else if ((uint8_t)pwr == 0x7F) {
                ESP_LOGW(DEVICE_NAME, "Remote TX POWER report: power not available (conn=0x%04X phy=%s reason=%u)",
                        param->trans_power_report_evt.conn_handle,
                        phy,
                        param->trans_power_report_evt.reason);
            } else {
                ESP_LOGI(DEVICE_NAME, "Remote TX POWER report: %d dBm (conn=0x%04X phy=%s reason=%u delta=%d flag=0x%02X)",
                        pwr,
                        param->trans_power_report_evt.conn_handle,
                        phy,
                        param->trans_power_report_evt.reason,
                        param->trans_power_report_evt.delta,
                        param->trans_power_report_evt.tx_power_level_flag);
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
        ESP_LOGI(DEVICE_NAME, "REG_EVT ok, set scan params");
        esp_err_t err = esp_ble_gap_set_scan_params(&ble_scan_params);
        if (err != ESP_OK) {
            ESP_LOGE(DEVICE_NAME, "set_scan_params failed: %s", esp_err_to_name(err));
        }
        break;
    }

    case ESP_GATTC_OPEN_EVT:
        if (param->open.status != ESP_GATT_OK) {
            ESP_LOGE(DEVICE_NAME, "OPEN_EVT failed, status=%d", param->open.status);
            connect_ok = false;
            esp_ble_gap_start_scanning(0);
            break;
        }
        ESP_LOGI(DEVICE_NAME, "OPEN_EVT ok");
        break;

    case ESP_GATTC_CONNECT_EVT:
        ESP_LOGI(DEVICE_NAME, "CONNECT_EVT conn_id=%d", param->connect.conn_id);
        connect_ok = true;
        p->conn_id = param->connect.conn_id;
        memcpy(p->remote_bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));

        ble_rf_tune_on_connect(param->connect.remote_bda);

        esp_err_t ret = esp_ble_gap_read_phy(p->remote_bda);
        if (ret)
        {
            ESP_LOGE(DEVICE_NAME, "esp_ble_gap_read_phy failed");
        }

        // MTU request (opzionale)
        ret = esp_ble_gattc_send_mtu_req(gattc_if, p->conn_id);
        if (ret)
        {
            ESP_LOGE(DEVICE_NAME, "esp_ble_gattc_send_mtu_req failed");
        }

        ret = esp_ble_gap_read_rssi(p->remote_bda);
        if (ret)
        {
            ESP_LOGE(DEVICE_NAME, "esp_ble_gap_read_rssi failed");
        }

        esp_power_level_t tx_power = esp_ble_tx_power_get(
            ESP_BLE_PWR_TYPE_CONN_HDL0 + p->conn_id
        );

        ESP_LOGI(DEVICE_NAME, "Local TX Power (conn_id=%d): %d dBm",
                 p->conn_id,
                 prettyprinter_get_tx_power_dbm(tx_power));

        ret = esp_ble_gap_read_remote_transmit_power_level(p->conn_id, ESP_BLE_CONN_TX_POWER_PHY_1M);
        if (ret)
        {
            ESP_LOGE(DEVICE_NAME, "esp_ble_gap_read_remote_transmit_power_level failed");
        }

        break;

    case ESP_GATTC_CFG_MTU_EVT:
        ESP_LOGI(DEVICE_NAME, "CFG_MTU_EVT mtu=%d", param->cfg_mtu.mtu);
        // avvia discovery dei servizi
        esp_ble_gattc_search_service(gattc_if, param->cfg_mtu.conn_id, NULL);
        break;

    case ESP_GATTC_SEARCH_RES_EVT: {
        // In ESP-IDF 5.5.1: search_res.srvc_id è esp_gatt_id_t
        esp_gatt_id_t *srvc_id = &param->search_res.srvc_id;

        if (srvc_id->uuid.len == ESP_UUID_LEN_16 &&
            srvc_id->uuid.uuid.uuid16 == REMOTE_SERVICE_UUID) {

            ESP_LOGI(DEVICE_NAME, "Found service 0x%04x", REMOTE_SERVICE_UUID);
            service_ok = true;
            p->service_start_handle = param->search_res.start_handle;
            p->service_end_handle   = param->search_res.end_handle;
        }
        break;
    }

    case ESP_GATTC_SEARCH_CMPL_EVT:
        if (!service_ok) {
            ESP_LOGW(DEVICE_NAME, "Service 0x%04x not found -> close", REMOTE_SERVICE_UUID);
            esp_ble_gattc_close(gattc_if, p->conn_id);
            break;
        }

        // Discover characteristic STREAM by UUID
        {
            uint16_t count = 0;
            esp_ble_gattc_get_attr_count(gattc_if, p->conn_id,
                                         ESP_GATT_DB_CHARACTERISTIC,
                                         p->service_start_handle,
                                         p->service_end_handle,
                                         INVALID_HANDLE,
                                         &count);

            if (count == 0) {
                ESP_LOGE(DEVICE_NAME, "No characteristics found in service");
                break;
            }

            char_elem_result = (esp_gattc_char_elem_t *)malloc(sizeof(esp_gattc_char_elem_t) * count);
            if (!char_elem_result) {
                ESP_LOGE(DEVICE_NAME, "malloc char_elem_result failed");
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
                ESP_LOGI(DEVICE_NAME, "STREAM char handle=0x%04x -> register notify", p->char_handle);
                esp_ble_gattc_register_for_notify(gattc_if, p->remote_bda, p->char_handle);
            } else {
                ESP_LOGE(DEVICE_NAME, "STREAM characteristic not found");
            }

            free(char_elem_result);
            char_elem_result = NULL;
        }

        // Discover characteristic LED (WRITE) by UUID (...0000) - serve per comandare lo stato LED sul server
        {
            uint16_t count = 0;
            esp_ble_gattc_get_attr_count(gattc_if, p->conn_id,
                                         ESP_GATT_DB_CHARACTERISTIC,
                                         p->service_start_handle,
                                         p->service_end_handle,
                                         INVALID_HANDLE,
                                         &count);

            if (count > 0) {
                esp_gattc_char_elem_t *tmp = (esp_gattc_char_elem_t *)malloc(sizeof(esp_gattc_char_elem_t) * count);
                if (!tmp) {
                    ESP_LOGE(DEVICE_NAME, "malloc led char elem failed");
                } else {
                    esp_ble_gattc_get_char_by_uuid(gattc_if, p->conn_id,
                                                   p->service_start_handle,
                                                   p->service_end_handle,
                                                   remote_led_char_uuid,
                                                   tmp,
                                                   &count);
                    if (count > 0) {
                        p->led_char_handle = tmp[0].char_handle;
                        ESP_LOGI(DEVICE_NAME, "LED char handle=0x%04x (WRITE)", p->led_char_handle);
                    } else {
                        ESP_LOGW(DEVICE_NAME, "LED characteristic not found (UUID ...0000)");
                    }
                    free(tmp);
                }
            } else {
                ESP_LOGW(DEVICE_NAME, "No characteristics in service while looking for LED char");
            }
        }

        break;

    case ESP_GATTC_REG_FOR_NOTIFY_EVT: {
        if (param->reg_for_notify.status != ESP_GATT_OK) {
            ESP_LOGE(DEVICE_NAME, "REG_FOR_NOTIFY failed, status=%d", param->reg_for_notify.status);
            break;
        }

        // Find CCCD descriptor (0x2902)
        uint16_t count = 0;
        esp_ble_gattc_get_attr_count(gattc_if, p->conn_id,
                                     ESP_GATT_DB_DESCRIPTOR,
                                     p->service_start_handle,
                                     p->service_end_handle,
                                     p->char_handle,
                                     &count);

        if (count == 0) {
            ESP_LOGE(DEVICE_NAME, "No descriptors found for STREAM char");
            break;
        }

        descr_elem_result = (esp_gattc_descr_elem_t *)malloc(sizeof(esp_gattc_descr_elem_t) * count);
        if (!descr_elem_result) {
            ESP_LOGE(DEVICE_NAME, "malloc descr_elem_result failed");
            break;
        }

        esp_ble_gattc_get_descr_by_char_handle(gattc_if,
                                              p->conn_id,
                                              p->char_handle,
                                              notify_descr_uuid,
                                              descr_elem_result,
                                              &count);

        if (count > 0) {
            p->cccd_handle = descr_elem_result[0].handle; // in IDF 5.5.x è "handle"
            ESP_LOGI(DEVICE_NAME, "CCCD handle=0x%04x (usa i pulsanti)", p->cccd_handle);
        } else {
            ESP_LOGE(DEVICE_NAME, "CCCD (0x2902) not found");
        }

        free(descr_elem_result);
        descr_elem_result = NULL;
        break;
    }

    case ESP_GATTC_WRITE_DESCR_EVT:
        if (param->write.status != ESP_GATT_OK) {
            ESP_LOGE(DEVICE_NAME, "WRITE_DESCR_EVT failed, status=0x%x", param->write.status);
            break;
        }
        ESP_LOGI(DEVICE_NAME, "WRITE_DESCR_EVT ok (handle=0x%04x)", param->write.handle);
        break;

    case ESP_GATTC_NOTIFY_EVT: {
        // ---------- NOTIFY stats (PRO) ----------
        int64_t now_us = esp_timer_get_time();

        // STREAM duration: aggiorna start/last/counters
        if (stream_t.active) {
            if (stream_t.start_us < 0) {
                stream_t.start_us = now_us;   // primo NOTIFY
            }
            stream_t.last_us = now_us;        // ultimo NOTIFY visto
            stream_t.packets++;
            stream_t.bytes += param->notify.value_len;
        }

        if (notify_stats.last_us >= 0) {
            int64_t delta_us = now_us - notify_stats.last_us;

            notify_stats.count++;
            notify_stats.sum_us += delta_us;

            if (delta_us < notify_stats.min_us) notify_stats.min_us = delta_us;
            if (delta_us > notify_stats.max_us) notify_stats.max_us = delta_us;
        } else {
            // primo pacchetto: inizia finestra
            notify_stats.window_start_us = now_us;
        }

        notify_stats.last_us = now_us;

        // stampa UNA volta al secondo per minimizzare overhead
        if ((now_us - notify_stats.window_start_us) >= 1000000LL) {

            if (notify_stats.count > 0) {
                int64_t avg_us = notify_stats.sum_us / (int64_t)notify_stats.count;
                int64_t jitter_us = notify_stats.max_us - notify_stats.min_us;

                ESP_LOGI(DEVICE_NAME,
                    "NOTIFY stats (1s): cnt=%" PRIu32
                    " avg=%" PRId64 " us (%.2f ms)"
                    " min=%" PRId64 " us"
                    " max=%" PRId64 " us"
                    " jitter=%" PRId64 " us",
                    notify_stats.count,
                    avg_us, avg_us / 1000.0,
                    notify_stats.min_us,
                    notify_stats.max_us,
                    jitter_us
                );
            }

            // reset finestra
            notify_stats.count = 0;
            notify_stats.sum_us = 0;
            notify_stats.min_us = INT64_MAX;
            notify_stats.max_us = 0;
            notify_stats.window_start_us = now_us;
        }

        // ---------- tua logica esistente ----------
        bool loss, wrong_data_len, counter_out_of_bound;
        dataloss_new_sample(param->notify.value, param->notify.value_len, &loss, &wrong_data_len, &counter_out_of_bound);

        if (counter_out_of_bound)
        {
            ESP_LOGI(DEVICE_NAME, "Dataloss percentage %f%%", dataloss_get_loss_percentage());
            ESP_LOGI(DEVICE_NAME, "Number of losses %u", dataloss_number_losses());
            ESP_LOGI(DEVICE_NAME, "DISABLE -> disable notify");
            cccd_write(0x0000);
        }
        else
        {
            if (loss)
            {
                int num_samples_lost = dataloss_get_last_loss_amplitude();
                ESP_LOGW(DEVICE_NAME, "DATA LOSS, lost %d samples", num_samples_lost);
            }

            if (wrong_data_len)
            {
                ESP_LOGW(DEVICE_NAME, "DATA LOSS, received only %d bytes", param->notify.value_len);
            }

            // Comanda il LED del SERVER in stato DATALOSS (scrive 0x06 sulla char LED).
            // Throttle: evita di spammare WRITE ad ogni NOTIFY se il problema persiste.
            if (loss || wrong_data_len) {
                int64_t now_us = esp_timer_get_time();
                if (s_last_led_dataloss_cmd_us < 0 || (now_us - s_last_led_dataloss_cmd_us) > (LED_THROTTLING_MS * 1000)) { // 2s
                    s_last_led_dataloss_cmd_us = now_us;
                    led_write_state(LED_DATALOSS);
                }
            }
        }

        break;
    }

    case ESP_GATTC_DISCONNECT_EVT:
        ESP_LOGI(DEVICE_NAME, "DISCONNECT reason=0x%02x", param->disconnect.reason);
        connect_ok = false;
        service_ok = false;
        p->service_start_handle = 0;
        p->service_end_handle = 0;
        p->char_handle = 0;
        p->cccd_handle = 0;
        p->led_char_handle = 0;
        s_last_led_dataloss_cmd_us = -1;

        // reset stats
        notify_stats.last_us = -1;
        notify_stats.count = 0;
        notify_stats.sum_us = 0;
        notify_stats.min_us = INT64_MAX;
        notify_stats.max_us = 0;
        notify_stats.window_start_us = 0;

        esp_ble_gap_start_scanning(0);
        break;

    default:
        break;
    }
}

// --------------------- app_main ---------------------

void app_main(void)
{
    memset(gl_profile_tab, 0, sizeof(gl_profile_tab));
    gl_profile_tab[PROFILE_A_APP_ID].gattc_cb = gattc_profile_event_handler;
    gl_profile_tab[PROFILE_A_APP_ID].gattc_if = ESP_GATT_IF_NONE;
    gl_profile_tab[PROFILE_A_APP_ID].app_id = PROFILE_A_APP_ID;

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
        ESP_LOGE(DEVICE_NAME, "bt_controller_init failed: %s", esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(DEVICE_NAME, "bt_controller_enable failed: %s", esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(DEVICE_NAME, "bluedroid_init failed: %s", esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(DEVICE_NAME, "bluedroid_enable failed: %s", esp_err_to_name(ret));
        return;
    }

    ble_rf_tune_before_adv();

    ESP_ERROR_CHECK(esp_ble_gap_set_device_name(DEVICE_NAME));

    ret = esp_ble_gatt_set_local_mtu(MTU);
    if (ret != ESP_OK) {
        ESP_LOGE(DEVICE_NAME, "set_local_mtu(%d) failed: %s", MTU, esp_err_to_name(ret));
    }

    ret = esp_ble_gap_register_callback(esp_gap_cb);
    if (ret) {
        ESP_LOGE(DEVICE_NAME, "gap_register_callback failed: %s", esp_err_to_name(ret));
        return;
    }

    ret = esp_ble_gattc_register_callback(esp_gattc_cb);
    if (ret) {
        ESP_LOGE(DEVICE_NAME, "gattc_register_callback failed: %s", esp_err_to_name(ret));
        return;
    }

    ret = esp_ble_gattc_app_register(PROFILE_A_APP_ID);
    if (ret) {
        ESP_LOGE(DEVICE_NAME, "gattc_app_register failed: %s", esp_err_to_name(ret));
        return;
    }

    // Pulsanti: scrivono CCCD quando disponibile
    xTaskCreate(button_task, "btn_task", 2048, NULL, 5, NULL);
}