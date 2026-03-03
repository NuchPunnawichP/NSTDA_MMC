// ═══════════════════════════════════════════════════════════════════════════════
//  hal_battery.cpp — Battery Monitoring Implementation
//
//  ADC → Voltage divider → actual battery voltage:
//    V_adc = ADC_raw / ADC_MAX * V_ref
//    V_bat = V_adc * DIVIDER_RATIO
//
//  EMA filter: avg = α × new + (1−α) × avg,  α = 0.1 (smooth)
// ═══════════════════════════════════════════════════════════════════════════════
#include "hal_battery.h"
#include "config.h"
#include <Arduino.h>

// ── EMA filter ──────────────────────────────────────────────────────────────
#define BATT_EMA_ALPHA_X256     26      // α ≈ 0.10 (26/256)
static uint16_t s_avg_mV = 0;
static bool     s_first  = true;

void hal_battery_init(void) {
    // ESP32-S3 ADC: configure pin as analog input
    analogReadResolution(ADC_RESOLUTION_BITS);
    // Use per-pin attenuation (global analogSetAttenuation may not work in core 3.x)
    analogSetPinAttenuation(PIN_BATTERY_ADC, ADC_11db);  // ~0-3.3V range
    pinMode(PIN_BATTERY_ADC, INPUT);

    // Prime the filter with first reading
    s_avg_mV = hal_battery_read_mV();
    s_first  = false;
}

uint16_t hal_battery_read_mV(void) {
    // analogReadMilliVolts() handles ADC calibration internally (core 3.x)
    // Returns voltage at ADC pin in mV (after attenuation, 0~3300mV)
    uint32_t v_adc_mV = analogReadMilliVolts(PIN_BATTERY_ADC);

    // Scale up by voltage divider ratio from config.h
    uint32_t v_bat_mV = (v_adc_mV * BATTERY_DIVIDER_X100) / 100;

    return (uint16_t)v_bat_mV;
}

void hal_battery_update(void) {
    uint16_t raw = hal_battery_read_mV();

    if (s_first) {
        s_avg_mV = raw;
        s_first  = false;
    } else {
        // EMA: avg = (α × raw + (256−α) × avg) / 256
        uint32_t a = BATT_EMA_ALPHA_X256;
        s_avg_mV = (uint16_t)((a * raw + (256 - a) * s_avg_mV) / 256);
    }
}

uint16_t hal_battery_get_avg_mV(void) {
    return s_avg_mV;
}

uint8_t hal_battery_get_level(void) {
    uint16_t v = s_avg_mV;
    if (v >= BATT_FULL_MV)     return BATT_LEVEL_FULL;
    if (v >= BATT_LOW_MV)      return BATT_LEVEL_NOMINAL;
    if (v >= BATT_CRITICAL_MV) return BATT_LEVEL_LOW;
    return BATT_LEVEL_CRITICAL;
}

bool hal_battery_is_critical(void) {
    return (s_avg_mV < BATT_CRITICAL_MV);
}
