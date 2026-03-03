// ═══════════════════════════════════════════════════════════════════════════════
//  hal_ui.cpp — User Interface HAL Implementation
//
//  WS2812: uses Arduino ESP32 core 3.x built-in neopixelWrite() (RMT-based)
//  Buttons: software debounce (~30ms)
//  DIP: direct GPIO read with internal pull-up
// ═══════════════════════════════════════════════════════════════════════════════
#include "hal_ui.h"
#include "config.h"
#include "hal_battery.h"
#include <Arduino.h>

// ── Debounce config ─────────────────────────────────────────────────────────
#define DEBOUNCE_MS             30
#define BTN_ACTIVE_LOW          true    // buttons pull LOW when pressed

// ── Button state struct ─────────────────────────────────────────────────────
typedef struct {
    uint8_t  pin;
    bool     current;       // debounced state
    bool     previous;      // previous debounced (for edge detect)
    bool     raw_prev;      // previous raw reading
    uint32_t last_change;   // millis of last raw change
} BtnState;

static BtnState s_btn_start;
static BtnState s_btn_mode;

// ── LED state ───────────────────────────────────────────────────────────────
static uint8_t s_led_r = 0, s_led_g = 0, s_led_b = 0;
static uint8_t s_led_brightness = 40;   // 0–255, default ~15% (WS2812 is BRIGHT)

// ── Helper: init a button ───────────────────────────────────────────────────
static void btn_init(BtnState* b, uint8_t pin) {
    b->pin         = pin;
    b->current     = false;
    b->previous    = false;
    b->raw_prev    = false;
    b->last_change = 0;
    pinMode(pin, INPUT_PULLUP);
}

// ── Helper: update button debounce ──────────────────────────────────────────
static void btn_update(BtnState* b) {
    bool raw = (digitalRead(b->pin) == LOW);  // active-low
    uint32_t now = millis();

    // Detect raw change → reset timer
    if (raw != b->raw_prev) {
        b->last_change = now;
        b->raw_prev = raw;
    }

    // If stable for DEBOUNCE_MS → accept
    if ((now - b->last_change) >= DEBOUNCE_MS) {
        b->previous = b->current;
        b->current  = raw;
    }
}

// ── Public API ──────────────────────────────────────────────────────────────

void hal_ui_init(void) {
    // LED pin
    pinMode(PIN_RGB_LED, OUTPUT);
    hal_ui_led_off();

    // Buttons
    btn_init(&s_btn_start, PIN_BUTTON_START);
    btn_init(&s_btn_mode,  PIN_BUTTON_MODE);

    // DIP switch (4 pins, pull-up, active-low)
    pinMode(PIN_DIP_SW_0, INPUT_PULLUP);
    pinMode(PIN_DIP_SW_1, INPUT_PULLUP);
    pinMode(PIN_DIP_SW_2, INPUT_PULLUP);
    pinMode(PIN_DIP_SW_3, INPUT_PULLUP);
}

// ── LED ─────────────────────────────────────────────────────────────────────

void hal_ui_led_set(uint8_t r, uint8_t g, uint8_t b) {
    s_led_r = r; s_led_g = g; s_led_b = b;
    // Scale by brightness (0–255)
    uint8_t br = s_led_brightness;
    uint8_t sr = (uint8_t)(((uint16_t)r * br) >> 8);
    uint8_t sg = (uint8_t)(((uint16_t)g * br) >> 8);
    uint8_t sb = (uint8_t)(((uint16_t)b * br) >> 8);
    neopixelWrite(PIN_RGB_LED, sr, sg, sb);
}

void hal_ui_led_brightness(uint8_t brightness) {
    s_led_brightness = brightness;
    // Re-apply current color with new brightness
    hal_ui_led_set(s_led_r, s_led_g, s_led_b);
}

uint8_t hal_ui_led_get_brightness(void) {
    return s_led_brightness;
}

void hal_ui_led_battery(uint8_t level) {
    switch (level) {
        case BATT_LEVEL_CRITICAL: hal_ui_led_set(255,   0,   0); break; // Red
        case BATT_LEVEL_LOW:      hal_ui_led_set(255, 180,   0); break; // Yellow
        case BATT_LEVEL_NOMINAL:  hal_ui_led_set(  0, 255,   0); break; // Green
        case BATT_LEVEL_FULL:     hal_ui_led_set(  0, 255,   0); break; // Green
        default:                  hal_ui_led_set( 50,  50,  50); break; // Dim white
    }
}

void hal_ui_led_off(void) {
    hal_ui_led_set(0, 0, 0);
}

// ── Buttons ─────────────────────────────────────────────────────────────────

void hal_ui_update(void) {
    btn_update(&s_btn_start);
    btn_update(&s_btn_mode);
}

bool hal_ui_btn_start(void)          { return s_btn_start.current; }
bool hal_ui_btn_mode(void)           { return s_btn_mode.current; }

bool hal_ui_btn_start_pressed(void)  { return  s_btn_start.current && !s_btn_start.previous; }
bool hal_ui_btn_start_released(void) { return !s_btn_start.current &&  s_btn_start.previous; }
bool hal_ui_btn_mode_pressed(void)   { return  s_btn_mode.current  && !s_btn_mode.previous; }
bool hal_ui_btn_mode_released(void)  { return !s_btn_mode.current  &&  s_btn_mode.previous; }

// ── DIP Switch ──────────────────────────────────────────────────────────────

uint8_t hal_ui_dip_read(void) {
    // Active-low: switch ON = GPIO LOW = bit set
    uint8_t val = 0;
    if (digitalRead(PIN_DIP_SW_0) == LOW) val |= (1 << 0);
    if (digitalRead(PIN_DIP_SW_1) == LOW) val |= (1 << 1);
    if (digitalRead(PIN_DIP_SW_2) == LOW) val |= (1 << 2);
    if (digitalRead(PIN_DIP_SW_3) == LOW) val |= (1 << 3);
    return val;
}
