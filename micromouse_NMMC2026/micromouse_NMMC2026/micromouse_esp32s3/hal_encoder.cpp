// ═══════════════════════════════════════════════════════════════════════════════
//  hal_encoder.cpp — Encoder HAL Implementation (ESP-IDF 5.x PCNT driver)
//
//  x4 Quadrature decoding:
//    Unit has 2 channels. Each channel watches one signal edge + the other
//    signal level. Together they capture all 4 transitions per cycle.
//
//  PCNT hardware handles counting in background — no ISR overhead.
//  Counter range: -32768 to +32767 per PCNT read.
//  We accumulate into int32_t software counters for extended range.
// ═══════════════════════════════════════════════════════════════════════════════
#include "hal_encoder.h"
#include "config.h"
#include <Arduino.h>
#include "driver/pulse_cnt.h"

// ── PCNT handles ────────────────────────────────────────────────────────────
static pcnt_unit_handle_t s_unitL = NULL;
static pcnt_unit_handle_t s_unitR = NULL;

// ── Software accumulators (extends 16-bit PCNT range) ───────────────────────
static volatile int32_t s_accumL = 0;
static volatile int32_t s_accumR = 0;

// ── Previous raw count for delta computation ────────────────────────────────
static int32_t s_prevL = 0;
static int32_t s_prevR = 0;

static bool s_init = false;

// ── Helper: configure one encoder unit with x4 quadrature ───────────────────
static bool encoder_init_unit(pcnt_unit_handle_t* unit,
                               int pin_a, int pin_b, int direction) {
    // Create PCNT unit
    pcnt_unit_config_t unit_cfg = {};
    unit_cfg.high_limit =  32767;
    unit_cfg.low_limit  = -32768;
    unit_cfg.flags.accum_count = 1;   // enable accumulator for extended range

    esp_err_t err = pcnt_new_unit(&unit_cfg, unit);
    if (err != ESP_OK) return false;

    // ── Channel 0: edge on A, level from B ──
    pcnt_chan_config_t chan0_cfg = {};
    chan0_cfg.edge_gpio_num  = pin_a;
    chan0_cfg.level_gpio_num = pin_b;
    pcnt_channel_handle_t chan0 = NULL;
    err = pcnt_new_channel(*unit, &chan0_cfg, &chan0);
    if (err != ESP_OK) return false;

    if (direction >= 0) {
        // A rising + B low → count UP (forward)
        pcnt_channel_set_edge_action(chan0,
            PCNT_CHANNEL_EDGE_ACTION_DECREASE,   // A falling
            PCNT_CHANNEL_EDGE_ACTION_INCREASE);  // A rising
        pcnt_channel_set_level_action(chan0,
            PCNT_CHANNEL_LEVEL_ACTION_KEEP,      // B low → keep direction
            PCNT_CHANNEL_LEVEL_ACTION_INVERSE);  // B high → invert
    } else {
        // Reversed direction
        pcnt_channel_set_edge_action(chan0,
            PCNT_CHANNEL_EDGE_ACTION_INCREASE,
            PCNT_CHANNEL_EDGE_ACTION_DECREASE);
        pcnt_channel_set_level_action(chan0,
            PCNT_CHANNEL_LEVEL_ACTION_KEEP,
            PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
    }

    // ── Channel 1: edge on B, level from A (completes x4) ──
    pcnt_chan_config_t chan1_cfg = {};
    chan1_cfg.edge_gpio_num  = pin_b;
    chan1_cfg.level_gpio_num = pin_a;
    pcnt_channel_handle_t chan1 = NULL;
    err = pcnt_new_channel(*unit, &chan1_cfg, &chan1);
    if (err != ESP_OK) return false;

    if (direction >= 0) {
        pcnt_channel_set_edge_action(chan1,
            PCNT_CHANNEL_EDGE_ACTION_INCREASE,
            PCNT_CHANNEL_EDGE_ACTION_DECREASE);
        pcnt_channel_set_level_action(chan1,
            PCNT_CHANNEL_LEVEL_ACTION_KEEP,
            PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
    } else {
        pcnt_channel_set_edge_action(chan1,
            PCNT_CHANNEL_EDGE_ACTION_DECREASE,
            PCNT_CHANNEL_EDGE_ACTION_INCREASE);
        pcnt_channel_set_level_action(chan1,
            PCNT_CHANNEL_LEVEL_ACTION_KEEP,
            PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
    }

    // ── Glitch filter (optional, helps with noisy signals) ──
    pcnt_glitch_filter_config_t filt_cfg = {};
    filt_cfg.max_glitch_ns = 1000;  // 1µs filter
    pcnt_unit_set_glitch_filter(*unit, &filt_cfg);

    // Enable and start
    pcnt_unit_enable(*unit);
    pcnt_unit_clear_count(*unit);
    pcnt_unit_start(*unit);

    return true;
}

// ── Helper: read accumulated count from a unit ──────────────────────────────
static int32_t encoder_read(pcnt_unit_handle_t unit) {
    int count = 0;
    pcnt_unit_get_count(unit, &count);
    return (int32_t)count;
}

// ── Public API ──────────────────────────────────────────────────────────────

bool hal_encoder_init(void) {
    bool ok = true;
    ok &= encoder_init_unit(&s_unitL,
                             PIN_ENC_L_A, PIN_ENC_L_B, ENC_L_DIRECTION);
    ok &= encoder_init_unit(&s_unitR,
                             PIN_ENC_R_A, PIN_ENC_R_B, ENC_R_DIRECTION);

    s_accumL = 0;
    s_accumR = 0;
    s_prevL  = 0;
    s_prevR  = 0;
    s_init   = ok;
    return ok;
}

int32_t hal_encoder_count_L(void) {
    if (!s_init) return 0;
    return encoder_read(s_unitL);
}

int32_t hal_encoder_count_R(void) {
    if (!s_init) return 0;
    return encoder_read(s_unitR);
}

int16_t hal_encoder_delta_L(void) {
    if (!s_init) return 0;
    int32_t now = encoder_read(s_unitL);
    int16_t d = (int16_t)(now - s_prevL);
    s_prevL = now;
    return d;
}

int16_t hal_encoder_delta_R(void) {
    if (!s_init) return 0;
    int32_t now = encoder_read(s_unitR);
    int16_t d = (int16_t)(now - s_prevR);
    s_prevR = now;
    return d;
}

void hal_encoder_reset(void) {
    if (!s_init) return;
    pcnt_unit_clear_count(s_unitL);
    pcnt_unit_clear_count(s_unitR);
    s_prevL = 0;
    s_prevR = 0;
}

void hal_encoder_reset_L(void) {
    if (!s_init) return;
    pcnt_unit_clear_count(s_unitL);
    s_prevL = 0;
}

void hal_encoder_reset_R(void) {
    if (!s_init) return;
    pcnt_unit_clear_count(s_unitR);
    s_prevR = 0;
}

bool hal_encoder_is_init(void) { return s_init; }
