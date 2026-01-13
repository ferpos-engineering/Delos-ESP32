/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
#ifndef LED_H
#define LED_H

/* Includes */
/* ESP APIs */
#include "driver/gpio.h"
#include "led_strip.h"

/* Defines */
#define CONFIG_EXAMPLE_BLINK_LED_STRIP 1
#define CONFIG_EXAMPLE_BLINK_LED_STRIP_BACKEND_RMT 1
#define CONFIG_EXAMPLE_BLINK_GPIO 8
#define BLINK_GPIO GPIO_NUM_8

// Number of LED states available
#define NUM_LED_STATES 10

// LED states

#define LED_OFF 0
#define LED_RED 1
#define LED_GREEN 2
#define LED_BLUE 3
#define LED_YELLOW 4
#define LED_BATTERY_LOW 5
#define LED_DATALOSS 6
#define LED_DISCONNECTED 7
#define LED_CONNECTED 8
#define LED_DATA_ACQUISITION 9

/**
 * @brief Initialize LED hardware and internal timers.
 *
 * - Initializes the configured backend (addressable strip via RMT/SPI, or GPIO LED).
 * - Creates internal esp_timer instances used for blinking and state expiration.
 * - Leaves the LED in LED_OFF.
 *
 * Call this once at boot before using led_on()/led_off().
 */
void led_init(void);

/**
 * @brief Activate a LED state (color + optional blink) using the state table configuration.
 *
 * Semantics driven by led_states[state_num]:
 * - blink_frequency_hz == 0  -> solid color (no blinking)
 * - blink_frequency_hz  > 0  -> blink at the given frequency (Hz)
 * - duration_ms == 0         -> infinite duration (persistent "base" state)
 * - duration_ms  > 0         -> temporary override state
 *
 * Stacking rules (preemption + restore):
 * - If you call led_on(TEMP) while another state is active, TEMP overrides it.
 * - When TEMP expires, the driver restores the previous still-valid state.
 * - If that previous state also expired while covered, the next one below is restored, etc.
 *
 * Special case:
 * - state_num == LED_OFF -> same as led_off()
 */
void led_on(uint8_t state_num);

/**
 * @brief Force the LED to OFF immediately.
 *
 * This clears the entire state stack and stops any ongoing blinking and/or timeout timers.
 */
void led_off(void);

/**
 * @brief Get the currently applied LED state (top of the state stack).
 *
 * This is the state that is effectively driving the LED right now.
 * - If the LED is blinking, this still returns the logical state (e.g. LED_DATA_ACQUISITION),
 *   not the instantaneous ON/OFF phase.
 * - If no state is active, returns LED_OFF.
 */
uint8_t get_led_state(void);

#endif // LED_H
