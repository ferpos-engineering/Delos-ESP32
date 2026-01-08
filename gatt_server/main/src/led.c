/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
/* Includes */
#include "led.h"

#include <inttypes.h>
#include <stdbool.h>

#include "esp_log.h"
#include "common.h"

#ifdef CONFIG_EXAMPLE_BLINK_LED_STRIP
#include "esp_timer.h"
#endif

struct led_states_t {
    uint32_t red;
    uint32_t green;
    uint32_t blue;
    float blink_frequency_hz; // set 0 to disable blinking
    uint32_t duration_ms;     // set 0 for infinite time.
};

/* Private variables */

static struct led_states_t led_states[NUM_LED_STATES] = {
    [LED_OFF] = { .red = 0, .green = 0, .blue = 0, .blink_frequency_hz = 0, .duration_ms = 0 },
    [LED_RED] = { .red = 255, .green = 0, .blue = 0, .blink_frequency_hz = 0, .duration_ms = 0 },
    [LED_GREEN] = { .red = 0, .green = 255, .blue = 0, .blink_frequency_hz = 0, .duration_ms = 0 },
    [LED_BLUE] = { .red = 0, .green = 0, .blue = 255, .blink_frequency_hz = 0, .duration_ms = 0 },
    [LED_YELLOW] = { .red = 255, .green = 255, .blue = 0, .blink_frequency_hz = 0, .duration_ms = 0 },
    [LED_BATTERY_LOW] = { .red = 255, .green = 0, .blue = 0, .blink_frequency_hz = 1.0, .duration_ms = 0 },
    [LED_DATALOSS] = { .red = 255, .green = 0, .blue = 0, .blink_frequency_hz = 4.0, .duration_ms = 2000 },
    [LED_DISCONNECTED] = { .red = 255, .green = 255, .blue = 255, .blink_frequency_hz = 0.5, .duration_ms = 0 },
    [LED_CONNECTED] = { .red = 0, .green = 0, .blue = 255, .blink_frequency_hz = 0.5, .duration_ms = 0 },
    [LED_DATA_ACQUISITION] = { .red = 0, .green = 255, .blue = 0, .blink_frequency_hz = 2.0, .duration_ms = 0 },
};

#ifdef CONFIG_EXAMPLE_BLINK_LED_STRIP
static led_strip_handle_t led_strip;

// --- blink engine (STRIP) ---
static esp_timer_handle_t s_toggle_timer = NULL; // periodic, toggles ON/OFF
static esp_timer_handle_t s_stop_timer = NULL;   // one-shot, stops after duration

static uint8_t s_active_state = LED_OFF;
static bool s_blinking = false;
static bool s_phase_on = false;

static void strip_apply_state(uint8_t state_num)
{
    const struct led_states_t *s = &led_states[state_num];
    led_strip_set_pixel(led_strip, 0, s->red, s->green, s->blue);
    led_strip_refresh(led_strip);
}

static void strip_clear(void)
{
    led_strip_clear(led_strip);
}

static void stop_timers_if_running(void)
{
    if (s_toggle_timer) {
        (void)esp_timer_stop(s_toggle_timer);
    }
    if (s_stop_timer) {
        (void)esp_timer_stop(s_stop_timer);
    }
    s_blinking = false;
    s_phase_on = false;
}

static void toggle_cb(void *arg)
{
    (void)arg;

    if (!s_blinking) {
        return;
    }

    s_phase_on = !s_phase_on;
    if (s_phase_on) {
        strip_apply_state(s_active_state);
    } else {
        strip_clear();
    }
}

static void stop_cb(void *arg)
{
    (void)arg;

    // stop periodic toggling and turn LED off
    stop_timers_if_running();
    strip_clear();
    s_active_state = LED_OFF;
}

static esp_err_t ensure_timers_created(void)
{
    if (!s_toggle_timer) {
        const esp_timer_create_args_t args = {
            .callback = &toggle_cb,
            .arg = NULL,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "led_toggle",
            .skip_unhandled_events = true,
        };
        esp_err_t err = esp_timer_create(&args, &s_toggle_timer);
        if (err != ESP_OK) {
            return err;
        }
    }

    if (!s_stop_timer) {
        const esp_timer_create_args_t args = {
            .callback = &stop_cb,
            .arg = NULL,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "led_stop",
            .skip_unhandled_events = true,
        };
        esp_err_t err = esp_timer_create(&args, &s_stop_timer);
        if (err != ESP_OK) {
            return err;
        }
    }

    return ESP_OK;
}

static void start_optional_stop_timer(uint32_t duration_ms)
{
    if (duration_ms == 0) {
        return; // infinite
    }
    // one-shot: duration_ms -> us
    ESP_ERROR_CHECK(esp_timer_start_once(s_stop_timer, (uint64_t)duration_ms * 1000ULL));
}
#endif

/* Public functions */
uint8_t get_led_state(void)
{
    return s_active_state;
}

#ifdef CONFIG_EXAMPLE_BLINK_LED_STRIP

void led_on(uint8_t state_num)
{
    if (state_num >= NUM_LED_STATES) {
        ESP_LOGW("LED", "state_num %u not supported", (unsigned)state_num);
        return;
    }

    // Always cancel previous blink / stop timer when changing state
    stop_timers_if_running();

    const struct led_states_t *st = &led_states[state_num];

    ESP_LOGI("LED",
             "led_on state=%u R=%" PRIu32 ", G=%" PRIu32 ", B=%" PRIu32 ", f=%.2fHz, duration=%" PRIu32 "ms",
             (unsigned)state_num, st->red, st->green, st->blue, (double)st->blink_frequency_hz, st->duration_ms);

    // OFF means: clear and return
    if (state_num == LED_OFF) {
        strip_clear();
        s_active_state = LED_OFF;
        return;
    }

    // Ensure timers exist (only needed if we might blink or time-out)
    // We also use stop timer for fixed ON with finite duration.
    if (ensure_timers_created() != ESP_OK) {
        // fallback: fixed ON without timers
        strip_apply_state(state_num);
        s_active_state = state_num;
        return;
    }

    s_active_state = state_num;

    // If frequency is 0 -> fixed ON
    if (!(st->blink_frequency_hz > 0.0f)) {
        strip_apply_state(state_num);
        start_optional_stop_timer(st->duration_ms);
        return;
    }

    // Blinking
    // half-period in us: toggle ON/OFF each half period => 0.5 / f seconds
    double half_period_us_d = 500000.0 / (double)st->blink_frequency_hz;
    uint64_t half_period_us = (uint64_t)(half_period_us_d + 0.5);

    // safety clamp
    if (half_period_us < 1000) {
        half_period_us = 1000; // 1ms
    }
    if (half_period_us > 5ULL * 1000 * 1000) {
        half_period_us = 5ULL * 1000 * 1000; // 5s
    }

    // Start in ON phase
    s_blinking = true;
    s_phase_on = true;
    strip_apply_state(state_num);

    ESP_ERROR_CHECK(esp_timer_start_periodic(s_toggle_timer, half_period_us));
    start_optional_stop_timer(st->duration_ms);
}

void led_off(void)
{
    stop_timers_if_running();
    strip_clear();
    s_active_state = LED_OFF;
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
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
#elif CONFIG_EXAMPLE_BLINK_LED_STRIP_BACKEND_SPI
    led_strip_spi_config_t spi_config = {
        .spi_bus = SPI2_HOST,
        .flags.with_dma = true,
    };
    ESP_ERROR_CHECK(led_strip_new_spi_device(&strip_config, &spi_config, &led_strip));
#else
#error "unsupported LED strip backend"
#endif

    // Create timers once (optional, but helps surface errors early)
    ESP_ERROR_CHECK(ensure_timers_created());

    /* Set all LED off to clear all pixels */
    led_off();
}

#elif CONFIG_EXAMPLE_BLINK_LED_GPIO

// NOTE: This branch keeps the original signature (no state_num).
// If you also want timed blink on the GPIO LED, tell me and I will adapt it.

void led_on(void)
{
    gpio_set_level(CONFIG_EXAMPLE_BLINK_GPIO, false);
    led_is_on = true;
}

void led_off(void)
{
    gpio_set_level(CONFIG_EXAMPLE_BLINK_GPIO, true);
    led_is_on = false;
}

void led_init(void)
{
    gpio_reset_pin(CONFIG_EXAMPLE_BLINK_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(CONFIG_EXAMPLE_BLINK_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(CONFIG_EXAMPLE_BLINK_GPIO, 1);
    led_is_on = false;
}

#else
#error "unsupported LED type"
#endif
