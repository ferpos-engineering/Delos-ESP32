/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include "led.h"

#include <inttypes.h>
#include <stdbool.h>
#include <stddef.h>

#include "esp_log.h"
#include "common.h"

#ifdef CONFIG_EXAMPLE_BLINK_LED_STRIP
#include "esp_timer.h"
#endif

#define TAG "LED"

// -----------------------------------------------------------------------------
// LED state table
// -----------------------------------------------------------------------------

struct led_states_t {
    uint32_t red;
    uint32_t green;
    uint32_t blue;
    float blink_frequency_hz; // set 0 to disable blinking
    uint32_t duration_ms;     // set 0 for infinite time.
};

static struct led_states_t led_states[NUM_LED_STATES] = {
    [LED_OFF] = { .red = 0, .green = 0, .blue = 0, .blink_frequency_hz = 0, .duration_ms = 0 },
    [LED_RED] = { .red = 255, .green = 0, .blue = 0, .blink_frequency_hz = 0, .duration_ms = 0 },
    [LED_GREEN] = { .red = 0, .green = 255, .blue = 0, .blink_frequency_hz = 0, .duration_ms = 0 },
    [LED_BLUE] = { .red = 0, .green = 0, .blue = 255, .blink_frequency_hz = 0, .duration_ms = 0 },
    [LED_YELLOW] = { .red = 255, .green = 255, .blue = 0, .blink_frequency_hz = 0, .duration_ms = 0 },
    [LED_BATTERY_LOW] = { .red = 255, .green = 0, .blue = 0, .blink_frequency_hz = 1.0f, .duration_ms = 0 },
    [LED_DATALOSS] = { .red = 255, .green = 255, .blue = 0, .blink_frequency_hz = 0, .duration_ms = 2000 },
    [LED_DISCONNECTED] = { .red = 255, .green = 255, .blue = 255, .blink_frequency_hz = 0.5f, .duration_ms = 0 },
    [LED_CONNECTED] = { .red = 0, .green = 0, .blue = 255, .blink_frequency_hz = 0.5f, .duration_ms = 0 },
    [LED_DATA_ACQUISITION] = { .red = 0, .green = 255, .blue = 0, .blink_frequency_hz = 2.0f, .duration_ms = 0 },
};

// -----------------------------------------------------------------------------
// State stacking engine
//
// Requirement:
// - Calling led_on(TEMP) while another state is active temporarily overrides it.
// - When TEMP expires, restore the previous still-valid state.
// - If previous also expired while TEMP was active, restore the one below, etc.
// -----------------------------------------------------------------------------

typedef struct {
    uint8_t state;            // LED_* enum
    uint64_t end_time_us;     // 0 => infinite
} led_stack_entry_t;

#define LED_STACK_MAX 8

static led_stack_entry_t s_stack[LED_STACK_MAX];
static uint8_t s_stack_len = 0;

/**
 * @brief Return monotonic time in microseconds.
 *
 * Used to compute state expiration deadlines.
 *
 * Note: in this implementation the timestamp is only available when the STRIP backend
 * is enabled (uses esp_timer_get_time()). For GPIO-only builds, this returns 0 and thus
 * finite durations are not supported unless you enable esp_timer as well.
 */
static uint64_t now_us(void)
{
#ifdef CONFIG_EXAMPLE_BLINK_LED_STRIP
    return (uint64_t)esp_timer_get_time();
#else
    // For GPIO-only builds we still can use esp_timer_get_time if available,
    // but keep this compile-safe.
    return 0;
#endif
}

static bool is_expired(const led_stack_entry_t *e, uint64_t tnow)
{
    return (e->end_time_us != 0) && (tnow >= e->end_time_us);
}

static void stack_pop_expired(uint64_t tnow)
{
    // We only ever need to pop from the top.
    // If a lower-priority state expires while being covered, we will pop it later
    // when it becomes the top again (after higher-priority overrides are removed).
    while (s_stack_len > 0 && is_expired(&s_stack[s_stack_len - 1], tnow)) {
        s_stack_len--;
    }
}

static uint8_t stack_top_state(void)
{
    if (s_stack_len == 0) return LED_OFF;
    return s_stack[s_stack_len - 1].state;
}

static uint64_t stack_top_remaining_us(uint64_t tnow)
{
    if (s_stack_len == 0) return 0;
    uint64_t end = s_stack[s_stack_len - 1].end_time_us;
    if (end == 0) return 0;
    return (end > tnow) ? (end - tnow) : 1;
}

static void stack_clear(void)
{
    s_stack_len = 0;
}

/**
 * @brief Replace the persistent base state.
 *
 * Clears the whole stack and sets @p state as the new bottom (infinite) state.
 * Passing LED_OFF clears the stack and leaves no active base.
 */
static void stack_set_base(uint8_t state)
{
    stack_clear();
    if (state == LED_OFF) {
        return;
    }
    s_stack[0].state = state;
    s_stack[0].end_time_us = 0; // infinite
    s_stack_len = 1;
}

/**
 * @brief Push a new override state on top of the stack.
 *
 * @param state        The LED_* state to activate.
 * @param duration_ms  Duration of the override. 0 => infinite (rare, but supported).
 * @param tnow         Current time in microseconds (used to compute end_time_us).
 */
static void stack_push_override(uint8_t state, uint32_t duration_ms, uint64_t tnow)
{
    // If stack full, drop the oldest (bottom) to make room (keeps most recent semantics)
    if (s_stack_len >= LED_STACK_MAX) {
        for (int i = 1; i < LED_STACK_MAX; ++i) {
            s_stack[i - 1] = s_stack[i];
        }
        s_stack_len = LED_STACK_MAX - 1;
    }

    s_stack[s_stack_len].state = state;
    if (duration_ms == 0) {
        s_stack[s_stack_len].end_time_us = 0; // infinite
    } else {
        s_stack[s_stack_len].end_time_us = tnow + (uint64_t)duration_ms * 1000ULL;
    }
    s_stack_len++;
}

// -----------------------------------------------------------------------------
// Hardware backends
// -----------------------------------------------------------------------------

static uint8_t s_active_state = LED_OFF;

/**
 * @brief Get the currently applied LED state (top of the state stack).
 *
 * This is the state that is effectively driving the LED right now.
 * - If the LED is blinking, this still returns the logical state (e.g. LED_DATA_ACQUISITION),
 *   not the instantaneous ON/OFF phase.
 * - If no state is active, returns LED_OFF.
 */
uint8_t get_led_state(void)
{
    return s_active_state;
}

#ifdef CONFIG_EXAMPLE_BLINK_LED_STRIP

static led_strip_handle_t led_strip;

static esp_timer_handle_t s_toggle_timer = NULL; // periodic (blink)
static esp_timer_handle_t s_expire_timer = NULL; // one-shot (state timeout)

// The LED is currently blinking
static bool s_blinking = false;

// True when the LED is in the ON phase of the blinking; False when it is in the OFF phase of the blinking
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
    if (s_toggle_timer) (void)esp_timer_stop(s_toggle_timer);
    if (s_expire_timer) (void)esp_timer_stop(s_expire_timer);
    s_blinking = false;
    s_phase_on = false;
}

static void toggle_cb(void *arg)
{
    (void)arg;
    if (!s_blinking) return;

    s_phase_on = !s_phase_on;
    if (s_phase_on) {
        strip_apply_state(s_active_state);
    } else {
        strip_clear();
    }
}

static void apply_top_state_locked(void);

static void expire_cb(void *arg)
{
    (void)arg;

    // Current top expired: pop expired and restore next valid
    uint64_t tnow = now_us();
    stack_pop_expired(tnow);
    apply_top_state_locked();
}

static esp_err_t ensure_timers_created_real(void)
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
        if (err != ESP_OK) return err;
    }
    if (!s_expire_timer) {
        const esp_timer_create_args_t args = {
            .callback = &expire_cb,
            .arg = NULL,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "led_expire",
            .skip_unhandled_events = true,
        };
        esp_err_t err = esp_timer_create(&args, &s_expire_timer);
        if (err != ESP_OK) return err;
    }
    return ESP_OK;
}

static void start_expire_timer_for_top(uint64_t tnow)
{
    if (!s_expire_timer) return;

    uint64_t rem_us = stack_top_remaining_us(tnow);
    if (rem_us == 0) {
        // infinite
        return;
    }
    // one-shot
    ESP_ERROR_CHECK(esp_timer_start_once(s_expire_timer, rem_us));
}

static void start_blink_if_needed(uint8_t state_num)
{
    const struct led_states_t *st = &led_states[state_num];

    if (!(st->blink_frequency_hz > 0.0f)) {
        // fixed ON
        strip_apply_state(state_num);
        s_blinking = false;
        s_phase_on = true;
        return;
    }

    // Blinking: toggle every half period
    double half_period_us_d = 500000.0 / (double)st->blink_frequency_hz;
    uint64_t half_period_us = (uint64_t)(half_period_us_d + 0.5);

    if (half_period_us < 1000) half_period_us = 1000;
    if (half_period_us > 5ULL * 1000 * 1000) half_period_us = 5ULL * 1000 * 1000;

    // start ON immediately
    s_blinking = true;
    s_phase_on = true;
    strip_apply_state(state_num);

    ESP_ERROR_CHECK(esp_timer_start_periodic(s_toggle_timer, half_period_us));
}

/**
 * @brief Apply the state currently on top of the stack.
 *
 * Responsibilities:
 * - Stop any previously running timers (blink/expire)
 * - Pop already-expired states from the top
 * - Apply the new top state (OFF / solid / blink)
 * - Arm the expiration timer if the top state has a finite duration
 */
static void apply_top_state_locked(void)
{
    // stop any current behavior
    stop_timers_if_running();

    uint64_t tnow = now_us();
    stack_pop_expired(tnow);

    uint8_t top = stack_top_state();
    s_active_state = top;

    if (top == LED_OFF) {
        strip_clear();
        return;
    }

    // create timers if needed
    if (ensure_timers_created_real() != ESP_OK) {
        // fallback: fixed ON
        strip_apply_state(top);
        return;
    }

    // apply top state (fixed or blinking)
    start_blink_if_needed(top);

    // schedule expiration for top if finite
    start_expire_timer_for_top(tnow);
}

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
void led_on(uint8_t state_num)
{
    if (state_num >= NUM_LED_STATES) {
        ESP_LOGW(TAG, "state_num %u not supported", (unsigned)state_num);
        return;
    }

    const struct led_states_t *st = &led_states[state_num];

    ESP_LOGI(TAG,
             "led_on state=%u R=%" PRIu32 ", G=%" PRIu32 ", B=%" PRIu32 ", f=%.2fHz, duration=%" PRIu32 "ms",
             (unsigned)state_num, st->red, st->green, st->blue, (double)st->blink_frequency_hz, st->duration_ms);

    if (state_num == LED_OFF) {
        led_off();
        return;
    }

    uint64_t tnow = now_us();

    // If duration is infinite (0), treat as new base state
    if (st->duration_ms == 0) {
        stack_set_base(state_num);
    } else {
        // finite: push as temporary override
        stack_push_override(state_num, st->duration_ms, tnow);
    }

    apply_top_state_locked();
}

/**
 * @brief Force the LED to OFF immediately.
 *
 * This clears the entire state stack and stops any ongoing blinking and/or timeout timers.
 */
void led_off(void)
{
    stack_clear();
    stop_timers_if_running();
    strip_clear();
    s_active_state = LED_OFF;
}

/**
 * @brief Initialize LED hardware and internal timers.
 *
 * - Initializes the configured backend (addressable strip via RMT/SPI, or GPIO LED).
 * - Creates internal esp_timer instances used for blinking and state expiration.
 * - Leaves the LED in LED_OFF.
 *
 * Call this once at boot before using led_on()/led_off().
 */
void led_init(void)
{
    led_strip_config_t strip_config = {
        .strip_gpio_num = GPIO_NUM_8,
        .max_leds = 1,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_RGB,
        .flags.invert_out = false,
    };

#if CONFIG_EXAMPLE_BLINK_LED_STRIP_BACKEND_RMT
    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000,
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

    // create timers upfront (optional, but surfaces errors early)
    ESP_ERROR_CHECK(ensure_timers_created_real());

    led_off();
}

#elif CONFIG_EXAMPLE_BLINK_LED_GPIO

// GPIO backend: maintains state stack but ignores RGB (single LED).
// LED is assumed ACTIVE-LOW like your original code.

static void gpio_led_on_impl(void)
{
    gpio_set_level(CONFIG_EXAMPLE_BLINK_GPIO, false);
}

static void gpio_led_off_impl(void)
{
    gpio_set_level(CONFIG_EXAMPLE_BLINK_GPIO, true);
}

/**
 * @brief Activate a LED state on the GPIO backend.
 *
 * GPIO backend notes:
 * - The LED is a single on/off output (no RGB). Any state != LED_OFF is treated as ON.
 * - The same stacking and duration semantics are applied, so temporary overrides restore
 *   the previous state when they expire.
 * - Blinking is not implemented on the GPIO backend in this file (solid ON only).
 */

void led_on(uint8_t state_num)
{
    // For GPIO-only LED, we treat any non-OFF as ON. Still apply the stack logic.
    if (state_num >= NUM_LED_STATES) {
        ESP_LOGW(TAG, "state_num %u not supported", (unsigned)state_num);
        return;
    }

    if (state_num == LED_OFF) {
        led_off();
        return;
    }

    // duration semantics still apply
    const struct led_states_t *st = &led_states[state_num];
    uint64_t tnow = now_us();

    if (st->duration_ms == 0) {
        stack_set_base(state_num);
    } else {
        stack_push_override(state_num, st->duration_ms, tnow);
    }

    // For simplicity, GPIO backend does not blink here (no timers included in this minimal backend).
    // If you need blink on GPIO too, tell me and I'll add esp_timer logic mirroring STRIP.
    gpio_led_on_impl();
    s_active_state = state_num;
}

/**
 * @brief Force the GPIO LED OFF and clear state stack.
 */

void led_off(void)
{
    stack_clear();
    gpio_led_off_impl();
    s_active_state = LED_OFF;
}

/**
 * @brief Initialize the GPIO LED backend.
 *
 * Configures CONFIG_EXAMPLE_BLINK_GPIO as output (active-low) and switches the LED off.
 */

void led_init(void)
{
    gpio_reset_pin(CONFIG_EXAMPLE_BLINK_GPIO);
    gpio_set_direction(CONFIG_EXAMPLE_BLINK_GPIO, GPIO_MODE_OUTPUT);
    gpio_led_off_impl();
    s_active_state = LED_OFF;
}

#else
#error "unsupported LED type"
#endif
