#include "prettyprinter.h"

int prettyprinter_get_tx_power_dbm(esp_power_level_t power_level)
{
    int result;
    switch(power_level)
    {
        case ESP_PWR_LVL_N15:
            result = -15;
        break;

        case ESP_PWR_LVL_N12:
            result = -12;
        break;

        case ESP_PWR_LVL_N9:
            result = -9;
        break;

        case ESP_PWR_LVL_N6:
            result = -6;
        break;

        case ESP_PWR_LVL_N3:
            result = -3;
        break;

        case ESP_PWR_LVL_N0:
            result = 0;
        break;

        case ESP_PWR_LVL_P3:
            result = +3;
        break;

        case ESP_PWR_LVL_P6:
            result = +6;
        break;

        case ESP_PWR_LVL_P9:
            result = +9;
        break;

        case ESP_PWR_LVL_P12:
            result = +12;
        break;

        case ESP_PWR_LVL_P15:
            result = +15;
        break;

        case ESP_PWR_LVL_P18:
            result = +18;
        break;

        case ESP_PWR_LVL_P20:
            result = +20;
        break;

        case ESP_PWR_LVL_INVALID:
            result = 0xffffffff;
        break;

        default:
            result = 0xffffffff;
        break;
    }

    return result;
}

void prettyprinter_print_phy(esp_ble_gap_phy_t phy, char* to_string, uint8_t len)
{
    switch(phy)
    {
        case ESP_BLE_GAP_PHY_1M:
            snprintf(to_string, len, "1M");
        break;

        case ESP_BLE_GAP_PHY_2M:
            snprintf(to_string, len, "2M");
        break;

        case ESP_BLE_GAP_PHY_CODED:
            snprintf(to_string, len, "Coded");
        break;
    }
}