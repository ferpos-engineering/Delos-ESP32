#include "include/nvs_manager.h"

#include <string.h>
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_log.h"

static const char *TAG = "NVS-MGR";

/**
 * MAC hardcoded dei peripherals (MODIFICA QUI).
 * Ogni MAC è 6 byte: {0xAA,0xBB,0xCC,0xDD,0xEE,0xFF}
 */
static const esp_bd_addr_t DEFAULT_MACS[NVS_MGR_MAX_PEERS] = {
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }, // peripheral #0 - mac0   
    { 0xe4, 0xb0, 0x63, 0x41, 0x09, 0x8a }, // peripheral #1 - mac1
    { 0xe4, 0xb0, 0x63, 0x41, 0x11, 0xbe }, // peripheral #2 - mac2
};

// Namespace e keys
static const char *NVS_NS = "ble_cfg";
static const char *KEYS[NVS_MGR_MAX_PEERS] = { "mac0", "mac1", "mac2" };

static esp_bd_addr_t peripherals_macs[NVS_MGR_MAX_PEERS];
static uint8_t num_peripherals_macs = 0;

static bool is_valid_mac(const esp_bd_addr_t mac)
{
    // Consideriamo non valido un MAC tutto 0 o tutto FF (tipici placeholder)
    bool all0 = true, allf = true;
    for (int i = 0; i < 6; i++) {
        all0 &= (mac[i] == 0x00);
        allf &= (mac[i] == 0xFF);
    }
    return !(all0 || allf);
}

static esp_err_t nvs_get_mac(nvs_handle_t h, const char *key, esp_bd_addr_t out_mac, bool *present)
{ 
    size_t len = 0;
    esp_err_t err = nvs_get_blob(h, key, NULL, &len);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        if (present) *present = false;
        memset(out_mac, 0, 6);
        return ESP_OK;
    }
    if (err != ESP_OK) return err;

    if (len != 6) {
        // Dato corrotto o vecchio formato: trattiamo come non presente
        if (present) *present = false;
        memset(out_mac, 0, 6);
        return ESP_OK;
    }

    err = nvs_get_blob(h, key, out_mac, &len);
    if (err != ESP_OK) return err;

    if (present) *present = is_valid_mac(out_mac);
    return ESP_OK;
}

#ifndef NO_NVS
static esp_err_t nvs_set_mac(nvs_handle_t h, const char *key, const esp_bd_addr_t mac)
{
    return nvs_set_blob(h, key, mac, 6);
}
#endif

esp_err_t nvs_manager_init_flash(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    return ret;
}

/**
 * @brief Carica i MAC target (0..2) da NVS.
 *
 * @param loaded (out, opzionale) true se MAC target sono stati trovati in NVS e caricati
 */
esp_err_t nvs_manager_load_target_macs(bool *loaded)
{
    memset(peripherals_macs, 0, sizeof(peripherals_macs));
    num_peripherals_macs = 0;
    if (loaded) *loaded = false;

    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NS, NVS_READONLY, &h);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        return ESP_OK; // nessun MAC salvato
    }
    if (err != ESP_OK) return err;

    bool any = false;

    for (int i = 0; i < NVS_MGR_MAX_PEERS; i++) {
        bool present = false;
        esp_bd_addr_t mac = {0};

        err = nvs_get_mac(h, KEYS[i], mac, &present);
        if (err != ESP_OK) {
            nvs_close(h);
            return err;
        }

        if (present) {
            memcpy(peripherals_macs[num_peripherals_macs], mac, 6);
            num_peripherals_macs++;
            any = true;
            ESP_LOGI(TAG, "Target %s is "ESP_BD_ADDR_STR, KEYS[i], ESP_BD_ADDR_HEX(mac));
        }
    }

    nvs_close(h);
    if (loaded) *loaded = any;
    return ESP_OK;
}

esp_err_t nvs_manager_bootstrap_defaults(bool *wrote)
{
    if (wrote) *wrote = false;

    #ifdef NO_NVS
    return ESP_OK;
    #else
    // Leggi stato attuale: presenti / assenti
    bool loaded = false;

    esp_err_t err = nvs_manager_load_target_macs(&loaded);
    if (err != ESP_OK) return err;

    if (loaded) {
        ESP_LOGI(TAG, "Target MACs loaded from NVS to RAM");
        return ESP_OK;
    }
    else
    {
        ESP_LOGI(TAG, "Target MACs not found in NVS, default MACs programming...");
    }

    // Nessun MAC presente -> scrivi i default hardcoded
    nvs_handle_t h;
    err = nvs_open(NVS_NS, NVS_READWRITE, &h);
    if (err != ESP_OK) return err;

    for (int i = 0; i < NVS_MGR_MAX_PEERS; i++) {
        err = nvs_set_mac(h, KEYS[i], DEFAULT_MACS[i]);
        if (err != ESP_OK) {
            nvs_close(h);
            return err;
        }
    }

    err = nvs_commit(h);
    nvs_close(h);

    if (err == ESP_OK) {
        if (wrote) *wrote = true;
        ESP_LOGI(TAG, "Bootstrapped %d default MACs into NVS (because none were present)", NVS_MGR_MAX_PEERS);
    }
    else
    {
        return err;
    }

    err = nvs_manager_load_target_macs(&loaded);
    if (err != ESP_OK) return err;

    if (loaded) {
        ESP_LOGI(TAG, "Default MACs loaded from NVS to RAM");
        return ESP_OK;
    }
    else
    {
        ESP_LOGE(TAG, "Cannot load default MACs from NVS to RAM");
    }

    return err;
    #endif
}

uint8_t nvs_manager_get_target_macs(const esp_bd_addr_t** target_macs)
{
    #ifdef NO_NVS
        if (target_macs) *target_macs = (const esp_bd_addr_t *)DEFAULT_MACS;
        return (uint8_t)NVS_MGR_MAX_PEERS;
    #else
        if (target_macs) *target_macs = (const esp_bd_addr_t *)peripherals_macs;
        return num_peripherals_macs;
    #endif
}

esp_err_t nvs_manager_clear_target_macs(void)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NS, NVS_READWRITE, &h);
    if (err == ESP_ERR_NVS_NOT_FOUND) return ESP_OK;
    if (err != ESP_OK) return err;

    for (int i = 0; i < NVS_MGR_MAX_PEERS; i++) {
        // nvs_erase_key ritorna NOT_FOUND se non esiste: ok
        (void)nvs_erase_key(h, KEYS[i]);
    }

    err = nvs_commit(h);
    nvs_close(h);
    return err;
}
