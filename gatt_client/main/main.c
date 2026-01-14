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

#include "include/dataloss.h"
#include "include/prettyprinter.h"

// --------------------- CONFIG ---------------------

#define PROFILE_NUM                 3
#define PROFILE_SLOT0_ID            0
#define PROFILE_SLOT1_ID            1
#define PROFILE_SLOT2_ID            2
#define INVALID_HANDLE              0

#define MTU                         23

#define REMOTE_DEVICE_NAME          "DPPS-DTS"
#define REMOTE_SERVICE_UUID         0x1815   // AUTO_IO_SVC_UUID nel tuo server
#define LED_DATALOSS                6
#define LED_THROTTLING_MS           2000

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

/**
 * MAC hardcoded dei peripherals (MODIFICA QUI).
 * Ogni MAC è 6 byte: {0xAA,0xBB,0xCC,0xDD,0xEE,0xFF}
 */
static const esp_bd_addr_t TARGET_MACS[PROFILE_NUM] = {
    { 0xcc, 0xba, 0x97, 0xf2, 0x86, 0xb6 }, // peripheral #0 - mac0   
    { 0xe4, 0xb0, 0x63, 0x41, 0x09, 0x8a }, // peripheral #1 - mac1
    { 0xe4, 0xb0, 0x63, 0x41, 0x11, 0xbe }, // peripheral #2 - mac2
};

// --------------------- Peers context ---------------------

typedef struct {
    bool            connected;       // true after CONNECT_EVT
    bool            service_ok;      // target service discovered
    bool            ready;           // CCCD handle found (can enable notify)
    bool            want_reconnect;  // set on disconnect to auto-reconnect

    esp_bd_addr_t       bda;
    esp_ble_addr_type_t addr_type;
    uint16_t            conn_id;

    uint16_t        service_start_handle;
    uint16_t        service_end_handle;
    uint16_t        char_handle;
    uint16_t        cccd_handle;

    // Handle della characteristic LED (WRITE) per comandare lo stato sul server
    uint16_t        led_char_handle;
    
    // throttling write LED_DATALOSS verso server
    int64_t         s_last_led_dataloss_cmd_us;

    int64_t         next_reconnect_try_us;
} peer_ctx_t;

static peer_ctx_t s_peers[PROFILE_NUM];

// connection manager: allow only one esp_ble_gattc_open at a time (avoid ESP_GATT_BUSY)
static volatile bool s_open_in_progress = false;

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

typedef struct {
    esp_gattc_cb_t gattc_cb;
    uint16_t gattc_if;
    uint16_t app_id;
} gattc_profile_inst;

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
    .uuid = {.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG},
};

static gattc_profile_inst gl_profile_tab[PROFILE_NUM];

// --------------------- NOTIFY stats ---------------------

typedef struct {
    int64_t last_us;

    uint32_t count;
    int64_t sum_us;
    int64_t min_us;
    int64_t max_us;

    int64_t window_start_us;
} notify_stats_t;

static notify_stats_t notify_stats_peers[PROFILE_NUM];

// --------------------- STREAM duration ---------------------

typedef struct {
    bool     active;
    int64_t  start_us;   // timestamp del primo NOTIFY ricevuto
    int64_t  last_us;    // timestamp dell'ultimo NOTIFY ricevuto
    uint32_t packets;
    uint64_t bytes;
} stream_timer_t;

static stream_timer_t stream_timers_peers[PROFILE_NUM];

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

static int get_slot_of_target_mac(const esp_bd_addr_t bda)
{
    for (uint8_t i = 0; i < PROFILE_NUM; i++) {
        if (memcmp(bda, TARGET_MACS[i], sizeof(esp_bd_addr_t)) == 0) return i;
    }
    return -1;
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

static void peer_set_gatt_state(int slot, esp_ble_addr_type_t addr_type)
{
    peer_ctx_t* peer = &s_peers[slot]; 

    memset(&peer, 0, sizeof(peer));
    peer->connected = false;
    peer->service_ok = false;
    peer->ready = false;
    peer->want_reconnect = true; // candidate wants connection
    memcpy(peer->bda, TARGET_MACS[slot], sizeof(esp_bd_addr_t));
    peer->addr_type = addr_type;
    peer->conn_id = 0xFFFF;
    peer->s_last_led_dataloss_cmd_us = -1;
    peer->next_reconnect_try_us = 0;
}

static void peer_reset_gatt_state(peer_ctx_t* peer)
{
    peer->connected = false;
    peer->service_ok = false;
    peer->ready = false;
    peer->conn_id = 0xFFFF;
    peer->service_start_handle = 0;
    peer->service_end_handle = 0;
    peer->char_handle = 0;
    peer->cccd_handle = 0;
}

static esp_err_t cccd_write(int slot, uint16_t value)
{
    ESP_LOGI(DEVICE_NAME, "CCCD write request: 0x%04x (peers=%d)", value, slot);

    peer_ctx_t* peer = &s_peers[slot];
    if (!peer->connected || !peer->ready) {
        ESP_LOGI(DEVICE_NAME, "CCCD write request failed: (peers=%d) connected=%d, ready=%d, cccd_handle=%u", slot, peer->connected, peer->ready, peer->cccd_handle);
        return ESP_OK;
    }

    ESP_LOGI(DEVICE_NAME, "CCCD write request: (peers=%d) connected=%d, ready=%d, cccd_handle=%u", slot, peer->connected, peer->ready, peer->cccd_handle);

    gattc_profile_inst* profile = &gl_profile_tab[slot];

    if (profile->gattc_if == ESP_GATT_IF_NONE) {
        ESP_LOGW(DEVICE_NAME, "CCCD write skipped (gattc_if not ready)");
        return ESP_FAIL;
    }

    // CCCD è 2 byte little-endian
    uint8_t cccd[2] = { (uint8_t)(value & 0xFF), (uint8_t)((value >> 8) & 0xFF) };

    esp_err_t err = esp_ble_gattc_write_char_descr(
        profile->gattc_if,
        peer->conn_id,
        peer->cccd_handle,
        sizeof(cccd),
        cccd,
        ESP_GATT_WRITE_TYPE_RSP,
        ESP_GATT_AUTH_REQ_NONE
    );

    if (err != ESP_OK) {
        ESP_LOGE(DEVICE_NAME, "CCCD write failed (peer=%d, conn_id=%u): %s", slot, peer->conn_id, esp_err_to_name(err));
        return ESP_FAIL;
    }

    dataloss_reset(slot);

    // STREAM duration: arma/reset su enable, stampa su disable
    stream_timer_t* stream_t = &stream_timers_peers[slot];
    if (value == 0x0001) {
        stream_t->active = true;
        stream_t->start_us = -1;
        stream_t->last_us  = -1;
        stream_t->packets  = 0;
        stream_t->bytes    = 0;
    } else if (value == 0x0000) {
        if (stream_t->active && stream_t->start_us >= 0 && stream_t->last_us >= 0) {
            int64_t dur_us = stream_t->last_us - stream_t->start_us;
            ESP_LOGI(DEVICE_NAME,
                    "STREAM duration: %.3f s, packets=%" PRIu32 ", bytes=%" PRIu64,
                    dur_us / 1000000.0, stream_t->packets, stream_t->bytes);
        } else {
            ESP_LOGW(DEVICE_NAME, "STREAM duration: no data received");
        }
        stream_t->active = false;
    }

    ESP_LOGI(DEVICE_NAME, "CCCD write requested: 0x%04x (peers=%d)", value, slot);

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
static esp_err_t led_write_state(int slot, uint8_t state_num)
{
    peer_ctx_t* peer = &s_peers[slot];
    gattc_profile_inst* profile = &gl_profile_tab[slot];
    if (profile->gattc_if == ESP_GATT_IF_NONE || peer->conn_id == 0xFFFF || peer->led_char_handle == 0) {
        ESP_LOGW(DEVICE_NAME, "LED write skipped (conn/service/handle not ready)");
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t v = state_num;
    // WRITE con risposta (più robusto); il server ha permessi READ|WRITE sulla char LED
    esp_err_t err = esp_ble_gattc_write_char(
        profile->gattc_if,
        peer->conn_id,
        peer->led_char_handle,
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

// --------------------- Tasks ---------------------

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
                for(int i = 0; i < PROFILE_NUM; i++)
                {
                    cccd_write(i, 0x0001);
                }
            }
        }

        if (prev_dis == 1 && dis == 0) {
            vTaskDelay(pdMS_TO_TICKS(BTN_DEBOUNCE_MS));
            if (gpio_get_level(BTN_DISABLE_GPIO) == 0) {
                ESP_LOGI(DEVICE_NAME, "BTN_DISABLE -> disable notify");
                for(int i = 0; i < PROFILE_NUM; i++)
                {
                    cccd_write(i, 0x0000);
                }
            }
        }

        prev_en  = en;
        prev_dis = dis;
        vTaskDelay(pdMS_TO_TICKS(BTN_POLL_MS));
    }
}

static void connect_manager_task(void *arg)
{
    while (1)
    {
        for (int i = 0; i < PROFILE_NUM; i++)
        {
            gattc_profile_inst* profile = &gl_profile_tab[i];
            if (profile->gattc_if == ESP_GATT_IF_NONE || s_open_in_progress)
            {
                continue;
            }

            int64_t now_us = esp_timer_get_time();

            int connected_cnt = 0;
            for (int i = 0; i < PROFILE_NUM; i++) {
                if (s_peers[i].connected) connected_cnt++;
            }

            if (connected_cnt < PROFILE_NUM) {
                for (int i = 0; i < PROFILE_NUM; i++) {
                    peer_ctx_t* peer = &s_peers[i];
                    if (peer->connected) continue;

                    if (peer->next_reconnect_try_us != 0 && now_us < peer->next_reconnect_try_us) {
                        continue;
                    }

                    if (peer->want_reconnect) {
                        esp_err_t err = esp_ble_gattc_open(profile->gattc_if, peer->bda, peer->addr_type, true);
                        if (err == ESP_OK) {
                            s_open_in_progress = true;
                            ESP_LOGI(DEVICE_NAME,
                                    "Connecting slot %d to %02X:%02X:%02X:%02X:%02X:%02X",
                                    i,
                                    peer->bda[0], peer->bda[1], peer->bda[2], peer->bda[3], peer->bda[4], peer->bda[5]);
                            break;
                        } else {
                            // busy/invalid state -> retry later
                            peer->next_reconnect_try_us = now_us + 1000 * 1000; // 1s backoff
                        }
                    }
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

// --------------------- Callbacks ---------------------

static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
static void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);
static void gattc_profile_event_handler(int slot, esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);
static void gattc_profile_slot0_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);
static void gattc_profile_slot1_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);
static void gattc_profile_slot2_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);


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
                    {
                        int slot = get_slot_of_target_mac(param->scan_rst.bda);
                        if (slot < 0)
                        {
                            break;
                        }

                        ESP_LOGI(DEVICE_NAME,
                                    "Found target MAC %02X:%02X:%02X:%02X:%02X:%02X -> connect",
                                    param->scan_rst.bda[0], param->scan_rst.bda[1], param->scan_rst.bda[2],
                                    param->scan_rst.bda[3], param->scan_rst.bda[4], param->scan_rst.bda[5]);

                        esp_ble_gattc_open(gl_profile_tab[slot].gattc_if,
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

static void gattc_profile_slot0_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t* param)
{
    gattc_profile_event_handler(PROFILE_SLOT0_ID, event, gattc_if, param);
}

static void gattc_profile_slot1_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t* param)
{
    gattc_profile_event_handler(PROFILE_SLOT1_ID, event, gattc_if, param);
}

static void gattc_profile_slot2_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t* param)
{
    gattc_profile_event_handler(PROFILE_SLOT2_ID, event, gattc_if, param);
}

static void gattc_profile_event_handler(int slot, esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t* param)
{
    peer_ctx_t* peer = &s_peers[slot];

    switch (event)
    {
    case ESP_GATTC_REG_EVT: {
        ESP_LOGI(DEVICE_NAME, "REG_EVT ok, set scan params");
        esp_err_t err = esp_ble_gap_set_scan_params(&ble_scan_params);
        if (err != ESP_OK) {
            ESP_LOGE(DEVICE_NAME, "set_scan_params failed: %s", esp_err_to_name(err));
        }
        break;
    }

    case ESP_GATTC_OPEN_EVT: {
        if (param->open.status != ESP_GATT_OK) {
            ESP_LOGW(DEVICE_NAME, "OPEN_EVT failed status=%d", param->open.status);

            peer_reset_gatt_state(peer);
            peer->want_reconnect = true;
            peer->next_reconnect_try_us = esp_timer_get_time() + 1000 * 1000;

            s_open_in_progress = false;
            break;
        }

        ESP_LOGI(DEVICE_NAME, "OPEN_EVT peer=%d, status=%d, conn_id=%d", slot, param->open.status, param->open.conn_id);
        s_open_in_progress = false;
        break;
    }

    case ESP_GATTC_CONNECT_EVT: {

        ESP_LOGI(DEVICE_NAME,
            "ESP_GATTC_CONNECT_EVT (conn_id=%d)",
            param->connect.conn_id
        );

        peer_set_gatt_state(slot, BLE_ADDR_TYPE_PUBLIC);

        ESP_LOGI(DEVICE_NAME, "CONNECT_EVT conn_id=%d", param->connect.conn_id);

        peer->connected = true;
        peer->want_reconnect = true;
        peer->conn_id = param->connect.conn_id;
        peer->service_ok = false;
        peer->ready = false;
        memcpy(peer->bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));

        ble_rf_tune_on_connect(param->connect.remote_bda);

        esp_err_t ret = esp_ble_gap_read_phy(peer->bda);
        if (ret)
        {
            ESP_LOGE(DEVICE_NAME, "esp_ble_gap_read_phy failed");
        }

        // MTU request (opzionale)
        ret = esp_ble_gattc_send_mtu_req(gattc_if, peer->conn_id);
        if (ret)
        {
            ESP_LOGE(DEVICE_NAME, "esp_ble_gattc_send_mtu_req failed");
        }

        ret = esp_ble_gap_read_rssi(peer->bda);
        if (ret)
        {
            ESP_LOGE(DEVICE_NAME, "esp_ble_gap_read_rssi failed");
        }

        esp_power_level_t tx_power = esp_ble_tx_power_get(
            ESP_BLE_PWR_TYPE_CONN_HDL0 + peer->conn_id
        );

        ESP_LOGI(DEVICE_NAME, "Local TX Power (conn_id=%d): %d dBm",
                 peer->conn_id,
                 prettyprinter_get_tx_power_dbm(tx_power));

        ret = esp_ble_gap_read_remote_transmit_power_level(peer->conn_id, ESP_BLE_CONN_TX_POWER_PHY_1M);
        if (ret)
        {
            ESP_LOGE(DEVICE_NAME, "esp_ble_gap_read_remote_transmit_power_level failed");
        }

        break;
    }

    case ESP_GATTC_CFG_MTU_EVT:
        ESP_LOGI(DEVICE_NAME, "CFG_MTU_EVT conn_id=%d mtu=%d", param->cfg_mtu.conn_id, param->cfg_mtu.mtu);
        ESP_ERROR_CHECK(esp_ble_gattc_search_service(gattc_if, param->cfg_mtu.conn_id, NULL));
        break;

    case ESP_GATTC_SEARCH_RES_EVT: {

        ESP_LOGI(DEVICE_NAME,
            "ESP_GATTC_SEARCH_RES_EVT (conn_id=%d)",
            param->search_res.conn_id
        );

        esp_gatt_id_t *srvc_id = &param->search_res.srvc_id;

        if (srvc_id->uuid.len == ESP_UUID_LEN_16 &&
            srvc_id->uuid.uuid.uuid16 == REMOTE_SERVICE_UUID) {

            ESP_LOGI(DEVICE_NAME, "Found service 0x%04x (peer %d, conn_id=%u)", REMOTE_SERVICE_UUID, slot, peer->conn_id);
            peer->service_ok = true;
            peer->service_start_handle = param->search_res.start_handle;
            peer->service_end_handle   = param->search_res.end_handle;
        }
        break;
    }

    case ESP_GATTC_SEARCH_CMPL_EVT: {

        ESP_LOGI(DEVICE_NAME,
            "ESP_GATTC_SEARCH_CMPL_EVT (conn_id=%d)",
            param->search_cmpl.conn_id
        );

        if (!peer->service_ok) {
            ESP_LOGW(DEVICE_NAME, "Service 0x%04x not found (peer %d) -> close", REMOTE_SERVICE_UUID, slot);
            esp_ble_gattc_close(gattc_if, peer->conn_id);
            break;
        }

        // Discover STREAM characteristic by UUID
        {
            uint16_t count = 0;

            esp_gatt_status_t status = esp_ble_gattc_get_attr_count( 
                gattc_if,
                param->search_cmpl.conn_id,
                ESP_GATT_DB_CHARACTERISTIC,
                peer->service_start_handle,
                peer->service_end_handle,
                INVALID_HANDLE,
                &count);
            if (status != ESP_GATT_OK){
                ESP_LOGE(DEVICE_NAME, "esp_ble_gattc_get_attr_count error");
                break;
            }

            if (count == 0) {
                ESP_LOGE(DEVICE_NAME, "No characteristics found in service (peer %d)", slot);
                break;
            }

            esp_gattc_char_elem_t *tmp = (esp_gattc_char_elem_t *)malloc(sizeof(esp_gattc_char_elem_t) * count);
            if (!tmp) {
                ESP_LOGE(DEVICE_NAME, "malloc char_elem_result failed");
                break;
            }

            ESP_ERROR_CHECK(esp_ble_gattc_get_char_by_uuid(
                                           gattc_if,
                                           param->search_cmpl.conn_id,
                                           peer->service_start_handle,
                                           peer->service_end_handle,
                                           remote_filter_char_uuid,
                                           tmp,
                                           &count));

            if (count > 0) {
                peer->char_handle = tmp[0].char_handle;
                ESP_LOGI(DEVICE_NAME, "STREAM char handle=%u (peer %d) -> register notify", peer->char_handle, slot);
                ESP_ERROR_CHECK(esp_ble_gattc_register_for_notify(gattc_if, peer->bda, peer->char_handle));
            } else {
                ESP_LOGE(DEVICE_NAME, "STREAM characteristic not found (peer %d)", slot);
            }

            free(tmp);
        }

        // Discover characteristic LED (WRITE) by UUID (...0000) - serve per comandare lo stato LED sul server
        {
            uint16_t count = 0;
            
            esp_gatt_status_t status = esp_ble_gattc_get_attr_count( 
                gattc_if,
                param->search_cmpl.conn_id,
                ESP_GATT_DB_CHARACTERISTIC,
                peer->service_start_handle,
                peer->service_end_handle,
                INVALID_HANDLE,
                &count);
            if (status != ESP_GATT_OK){
                ESP_LOGE(DEVICE_NAME, "esp_ble_gattc_get_attr_count error");
                break;
            }

            if (count > 0) {
                esp_gattc_char_elem_t *tmp = (esp_gattc_char_elem_t *)malloc(sizeof(esp_gattc_char_elem_t) * count);
                if (!tmp) {
                    ESP_LOGE(DEVICE_NAME, "malloc led char elem failed");
                } else {
                    ESP_ERROR_CHECK(esp_ble_gattc_get_char_by_uuid(
                                                   gattc_if,
                                                   param->search_cmpl.conn_id,
                                                   peer->service_start_handle,
                                                   peer->service_end_handle,
                                                   remote_led_char_uuid,
                                                   tmp,
                                                   &count));
                    if (count > 0) {
                        peer->led_char_handle = tmp[0].char_handle;
                        ESP_LOGI(DEVICE_NAME, "LED char handle=%u (peer %d) (WRITE)", peer->led_char_handle, slot);
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
    }

    case ESP_GATTC_REG_FOR_NOTIFY_EVT: {

        ESP_LOGI(DEVICE_NAME,
            "REG_FOR_NOTIFY Notify registered: (slot=%d) handle=%u status=%d char_handle=%u",
            slot,
            param->reg_for_notify.handle,
            param->reg_for_notify.status,
            peer->char_handle
        );

        if (param->reg_for_notify.status != ESP_GATT_OK) {
            ESP_LOGE(DEVICE_NAME, "REG_FOR_NOTIFY failed (peer %d), status=%d", slot, param->reg_for_notify.status);
            break;
        }

        // Find CCCD descriptor (0x2902)
        uint16_t count = 0;
        ESP_ERROR_CHECK(esp_ble_gattc_get_attr_count(gattc_if, peer->conn_id,
                                     ESP_GATT_DB_DESCRIPTOR,
                                     peer->service_start_handle,
                                     peer->service_end_handle,
                                     peer->char_handle,
                                     &count));

        if (count == 0) {
            ESP_LOGE(DEVICE_NAME, "No descriptors found for STREAM char (peer %d)", slot);
            break;
        }

        esp_gattc_descr_elem_t *tmp = (esp_gattc_descr_elem_t *)malloc(sizeof(esp_gattc_descr_elem_t) * count);
        if (!tmp) {
            ESP_LOGE(DEVICE_NAME, "malloc descr_elem_result failed");
            break;
        }

        ESP_ERROR_CHECK(esp_ble_gattc_get_descr_by_char_handle(gattc_if,
                                              peer->conn_id,
                                              peer->char_handle,
                                              notify_descr_uuid,
                                              tmp,
                                              &count));

        if (count > 0) {
            peer->cccd_handle = tmp[0].handle;
            peer->ready = true;
            ESP_LOGI(DEVICE_NAME, "CCCD handle=%u (peer %d) - READY", peer->cccd_handle, slot);
        } else {
            ESP_LOGE(DEVICE_NAME, "CCCD (0x2902) not found (peer %d)", slot);
        }

        free(tmp);
        break;
    }

    case ESP_GATTC_WRITE_DESCR_EVT:
        if (param->write.status != ESP_GATT_OK) {
            ESP_LOGE(DEVICE_NAME, "WRITE_DESCR_EVT failed, status=0x%x", param->write.status);
            break;
        }
        ESP_LOGI(DEVICE_NAME, "WRITE_DESCR_EVT ok (handle=%u)", param->write.handle);
        break;

    case ESP_GATTC_NOTIFY_EVT: {

        // ---------- NOTIFY stats ----------

        int64_t now_us = esp_timer_get_time();

        // STREAM duration: aggiorna start/last/counters
        stream_timer_t* stream_t = &stream_timers_peers[slot];
        if (stream_t->active) {
            if (stream_t->start_us < 0) {
                stream_t->start_us = now_us;   // primo NOTIFY
            }
            stream_t->last_us = now_us;        // ultimo NOTIFY visto
            stream_t->packets++;
            stream_t->bytes += param->notify.value_len;
        }

        notify_stats_t* notify_stats = &notify_stats_peers[slot];
        if (notify_stats->last_us >= 0) {
            int64_t delta_us = now_us - notify_stats->last_us;

            notify_stats->count++;
            notify_stats->sum_us += delta_us;

            if (delta_us < notify_stats->min_us) notify_stats->min_us = delta_us;
            if (delta_us > notify_stats->max_us) notify_stats->max_us = delta_us;
        } else {
            // primo pacchetto: inizia finestra
            notify_stats->window_start_us = now_us;
        }

        notify_stats->last_us = now_us;

        // stampa UNA volta al secondo per minimizzare overhead
        if ((now_us - notify_stats->window_start_us) >= 1000000LL) {

            if (notify_stats->count > 0) {
                int64_t avg_us = notify_stats->sum_us / (int64_t)notify_stats->count;
                int64_t jitter_us = notify_stats->max_us - notify_stats->min_us;

                ESP_LOGI(DEVICE_NAME,
                    "NOTIFY stats peer=%d (1s): cnt=%" PRIu32
                    " avg=%" PRId64 " us (%.2f ms)"
                    " min=%" PRId64 " us"
                    " max=%" PRId64 " us"
                    " jitter=%" PRId64 " us",
                    slot,
                    notify_stats->count,
                    avg_us, avg_us / 1000.0,
                    notify_stats->min_us,
                    notify_stats->max_us,
                    jitter_us
                );
            }

            // reset finestra
            notify_stats->count = 0;
            notify_stats->sum_us = 0;
            notify_stats->min_us = INT64_MAX;
            notify_stats->max_us = 0;
            notify_stats->window_start_us = now_us;
        }

        bool loss, wrong_data_len, counter_out_of_bound;
        dataloss_new_sample(slot, param->notify.value, param->notify.value_len, &loss, &wrong_data_len, &counter_out_of_bound);

        if (counter_out_of_bound)
        {
            ESP_LOGI(DEVICE_NAME, "peer=%d Dataloss percentage %f%%", slot, dataloss_get_loss_percentage(slot));
            ESP_LOGI(DEVICE_NAME, "peer=%d Number of losses %u", slot, dataloss_number_losses(slot));
            ESP_LOGI(DEVICE_NAME, "peer=%d DISABLE -> disable notify", slot );
            cccd_write(slot, 0x0000);
        }
        else
        {
            if (loss)
            {
                int num_samples_lost = dataloss_get_last_loss_amplitude(slot);
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
                if (peer->s_last_led_dataloss_cmd_us < 0 || (now_us - peer->s_last_led_dataloss_cmd_us) > (LED_THROTTLING_MS * 1000)) { // 2s
                    peer->s_last_led_dataloss_cmd_us = now_us;
                    led_write_state(slot, LED_DATALOSS);
                }
            }
        }

        break;
    }

    case ESP_GATTC_DISCONNECT_EVT: {

        ESP_LOGI(DEVICE_NAME, "DISCONNECT conn_id=%d reason=0x%02x", param->disconnect.conn_id, param->disconnect.reason);

        peer_reset_gatt_state(peer);
        peer->want_reconnect = true;
        peer->next_reconnect_try_us = esp_timer_get_time() + 1000 * 1000;
        ESP_LOGI(DEVICE_NAME, "Peer %d scheduled for auto-reconnect", slot);
        
        peer->service_start_handle = 0;
        peer->service_end_handle = 0;
        peer->char_handle = 0;
        peer->cccd_handle = 0;
        peer->led_char_handle = 0;
        peer->s_last_led_dataloss_cmd_us = -1;

        // reset stats
        notify_stats_t* notify_stats = &notify_stats_peers[slot];
        notify_stats->last_us = -1;
        notify_stats->count = 0;
        notify_stats->sum_us = 0;
        notify_stats->min_us = INT64_MAX;
        notify_stats->max_us = 0;
        notify_stats->window_start_us = 0;

        // Keep scanning so that returning peers are seen quickly
        esp_ble_gap_start_scanning(0);
        break;
    }

    default:
        break;
    }
}

// --------------------- app_main ---------------------

esp_err_t nvs_manager_init_flash(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    return ret;
}

void app_main(void)
{
    for(int i = 0; i < PROFILE_NUM; i++)
    {
        notify_stats_t* notify_stats = &notify_stats_peers[i];
        notify_stats->last_us = -1;
        notify_stats->count = 0;
        notify_stats->sum_us = 0;
        notify_stats->min_us = INT64_MAX;
        notify_stats->max_us = 0;
        notify_stats->window_start_us = 0;

		stream_timer_t* stream_timers = &stream_timers_peers[i];
		stream_timers->active = false;
		stream_timers->start_us = -1;
		stream_timers->last_us = -1;
		stream_timers->packets = 0;
		stream_timers->bytes = 0;

        gattc_profile_inst* profile = &gl_profile_tab[i];
        memset(profile, 0, sizeof(gattc_profile_inst));
        profile->gattc_if = ESP_GATT_IF_NONE;
        profile->app_id = i;

        peer_ctx_t* peer = &s_peers[i];
        peer->connected = false;
        peer->service_ok = false;
        peer->ready = false;
        peer->want_reconnect = false;

        memset(&peer->bda[0], 0, sizeof(esp_bd_addr_t));
        peer->addr_type = BLE_ADDR_TYPE_PUBLIC;
        peer->conn_id = 0;

        peer->service_start_handle = 0;
        peer->service_end_handle = 0;
        peer->char_handle = 0;
        peer->cccd_handle = 0;

        peer->led_char_handle = 0;
        peer->s_last_led_dataloss_cmd_us = 0;
        peer->next_reconnect_try_us = 0;
    }

    gl_profile_tab[PROFILE_SLOT0_ID].gattc_cb = gattc_profile_slot0_event_handler;
    gl_profile_tab[PROFILE_SLOT1_ID].gattc_cb = gattc_profile_slot1_event_handler;
    gl_profile_tab[PROFILE_SLOT2_ID].gattc_cb = gattc_profile_slot2_event_handler;

    dataloss_init();

    ESP_ERROR_CHECK(nvs_manager_init_flash());

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_err_t ret = esp_bt_controller_init(&bt_cfg);
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

    for(int i = 0; i < PROFILE_NUM; i++)
    {
        ret = esp_ble_gattc_app_register(i);
        if (ret) {
            ESP_LOGE(DEVICE_NAME, "gattc_app_register slot=%d failed: %s", i, esp_err_to_name(ret));
            return;
        }
    }

    // Pulsanti: scrivono CCCD quando disponibile
    xTaskCreate(button_task, "btn_task", 2048, NULL, 5, NULL);

    // Multi-connection + auto-reconnect manager
    xTaskCreate(connect_manager_task, "conn_mgr", 3072, NULL, 6, NULL);
}
