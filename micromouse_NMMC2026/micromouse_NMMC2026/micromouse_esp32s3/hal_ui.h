// ═══════════════════════════════════════════════════════════════════════════════
//  hal_ui.h — User Interface HAL (LED, Buttons, DIP Switch)
//
//  Layer: HAL
//  Dependencies: config.h
//
//  Components:
//    - WS2812 RGB LED (onboard, 1 pixel)
//    - 2 push buttons (START, MODE) with software debounce
//    - 4-pin DIP switch (16 modes)
// ═══════════════════════════════════════════════════════════════════════════════
#ifndef HAL_UI_H
#define HAL_UI_H

#include <stdint.h>
#include <stdbool.h>

// ── LED ─────────────────────────────────────────────────────────────────────
void hal_ui_init(void);

// Set RGB color (0–255 each). Scaled by current brightness.
void hal_ui_led_set(uint8_t r, uint8_t g, uint8_t b);

// Set global brightness 0–255 (default=40). Applies to all subsequent led_set calls.
void hal_ui_led_brightness(uint8_t brightness);
uint8_t hal_ui_led_get_brightness(void);

// Convenience: set LED from battery level (auto color)
void hal_ui_led_battery(uint8_t level);  // BATT_LEVEL_*

// Turn LED off
void hal_ui_led_off(void);

// ── Buttons ─────────────────────────────────────────────────────────────────
// Call hal_ui_update() periodically (e.g. every 10ms) for debounce.
void hal_ui_update(void);

// Current debounced state (true = pressed)
bool hal_ui_btn_start(void);
bool hal_ui_btn_mode(void);

// Edge detection (true only on the tick when button transitions)
bool hal_ui_btn_start_pressed(void);   // rising edge (just pressed)
bool hal_ui_btn_start_released(void);  // falling edge (just released)
bool hal_ui_btn_mode_pressed(void);
bool hal_ui_btn_mode_released(void);

// ── DIP Switch ──────────────────────────────────────────────────────────────
// Read 4-bit value (0–15). Reads GPIO directly (no debounce needed).
uint8_t hal_ui_dip_read(void);

#endif // HAL_UI_H
