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
#define BLINK_GPIO GPIO_NUM_8

/* Public function declarations */
uint8_t get_led_state(void);
void led_on(uint8_t state_num);
void led_off(void);
void led_init(void);

#endif // LED_H
