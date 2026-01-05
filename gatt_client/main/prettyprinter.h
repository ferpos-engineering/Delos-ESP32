#ifndef PRETTYPRINTER_H
#define PRETTYPRINTER_H

/* Includes */
#include "esp_bt.h"
#include "esp_gap_ble_api.h"

/* ESP APIs */

int prettyprinter_get_tx_power_dbm(esp_power_level_t power_level);

void prettyprinter_print_phy(esp_ble_gap_phy_t phy, char* to_string, uint8_t len);

void prettyprinter_print_phy2(esp_ble_tx_power_phy_t phy, char* to_string, uint8_t len);

#endif // PRETTYPRINTER_H

