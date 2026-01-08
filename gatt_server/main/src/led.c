/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
/* Includes */
#include "led.h"

#include <inttypes.h>

#include "esp_log.h"
#include "common.h"

#define NUM_LED_STATES 4
#define LED_OFF 0
#define LED_RED 1
#define LED_GREEN 2
#define LED_BLUE 3

struct led_states_t {
    uint32_t red;
    uint32_t green;
    uint32_t blue;
    uint32_t blink_frequency;
};

/* Private variables */
static uint8_t led_is_on;

static struct led_states_t led_states[NUM_LED_STATES] = {
    [LED_OFF] = { .red = 0, .green = 0, .blue = 0, .blink_frequency = 0 },
    [LED_RED] = { .red = 255, .green = 0, .blue = 0, .blink_frequency = 0 },
    [LED_GREEN] = { .red = 0, .green = 255, .blue = 0, .blink_frequency = 0 },
    [LED_BLUE] = { .red = 0, .green = 0, .blue = 255, .blink_frequency = 0 },
};

#ifdef CONFIG_EXAMPLE_BLINK_LED_STRIP
static led_strip_handle_t led_strip;
#endif

/* Public functions */
uint8_t is_led_on(void)
{
    return led_is_on;
}

#ifdef CONFIG_EXAMPLE_BLINK_LED_STRIP

void led_on(uint8_t state_num)
{
    if(state_num >= NUM_LED_STATES)
    {
        ESP_LOGW("LED", "state_num %" PRIu32 " not supported", state_num);
        return;
    }

    struct led_states_t led_state = led_states[state_num];

    ESP_LOGI("LED", "led_on R=%" PRIu32 ", G=%" PRIu32 ", B=%" PRIu32,
            led_state.red, led_state.green, led_state.blue);

    /* Set the LED pixel using RGB from 0 (0%) to 255 (100%) for each color */
    led_strip_set_pixel(led_strip, 0, led_state.red, led_state.green, led_state.blue);

    /* Refresh the strip to send data */
    led_strip_refresh(led_strip);

    /* Update LED state */
    led_is_on = true;
}

void led_off(void)
{
    /* Set all LED off to clear all pixels */
    led_strip_clear(led_strip);

    /* Update LED state */
    led_is_on = false;
}

void led_init(void)
{
    // ESP_LOGI(TAG, "example configured to blink addressable led!");
    /* LED strip initialization with the GPIO and pixels number*/
    led_strip_config_t strip_config = {
        .strip_gpio_num = GPIO_NUM_8,
        .max_leds = 1,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_RGB,
        .flags.invert_out = false,
    };

#if CONFIG_EXAMPLE_BLINK_LED_STRIP_BACKEND_RMT
    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000, // 10MHz
        .flags.with_dma = false,
    };
    ESP_ERROR_CHECK(
        led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
#elif CONFIG_EXAMPLE_BLINK_LED_STRIP_BACKEND_SPI
    led_strip_spi_config_t spi_config = {
        .spi_bus = SPI2_HOST,
        .flags.with_dma = true,
    };
    ESP_ERROR_CHECK(
        led_strip_new_spi_device(&strip_config, &spi_config, &led_strip));
#else
#error "unsupported LED strip backend"
#endif
    /* Set all LED off to clear all pixels */
    led_off();
}

#elif CONFIG_EXAMPLE_BLINK_LED_GPIO

void led_on(void)
{
    gpio_set_level(CONFIG_EXAMPLE_BLINK_GPIO, false);
}

void led_off(void)
{
    gpio_set_level(CONFIG_EXAMPLE_BLINK_GPIO, true);
}

void led_init(void)
{
    // ESP_LOGI(TAG, "example configured to blink gpio led!");
    gpio_reset_pin(CONFIG_EXAMPLE_BLINK_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(CONFIG_EXAMPLE_BLINK_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(CONFIG_EXAMPLE_BLINK_GPIO, 1);
}

#else
#error "unsupported LED type"
#endif
