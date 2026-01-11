#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "esp_bt_defs.h"   // esp_bd_addr_t

#ifdef __cplusplus
extern "C" {
#endif

#define NO_NVS
#define NVS_MGR_MAX_PEERS 3

/**
 * @brief Inizializza NVS (nvs_flash_init) se non già inizializzata dal chiamante.
 *
 * Se tu inizializzi già NVS in app_main(), puoi non chiamarla.
 * Questa funzione gestisce anche i casi classici:
 *  - ESP_ERR_NVS_NO_FREE_PAGES
 *  - ESP_ERR_NVS_NEW_VERSION_FOUND
 */
esp_err_t nvs_manager_init_flash(void);

/**
 * @brief Programma in NVS i MAC hardcoded (slot 0..2) SOLO se non già presenti.
 *        Se già presenti li carica da NVS in memoria.
 *
 * Logica:
 * - Se almeno un MAC è già presente in NVS -> non sovrascrive nulla (idempotente).
 * - Se nessun MAC è presente -> scrive i 3 MAC hardcoded e fa commit.
 *
 * @param wrote  (out, opzionale) true se ha scritto davvero in NVS.
 */
esp_err_t nvs_manager_bootstrap_defaults(bool *wrote);

uint8_t nvs_manager_get_target_macs(const esp_bd_addr_t** target_macs);

/**
 * @brief Cancella i MAC salvati in NVS (mac0/mac1/mac2).
 */
esp_err_t nvs_manager_clear_target_macs(void);

#ifdef __cplusplus
}
#endif
