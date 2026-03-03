// ═══════════════════════════════════════════════════════════════════════════════
//  hal_battery.h — Battery Monitoring HAL
//
//  Layer: HAL
//  Dependencies: config.h
//
//  Design:
//    - Reads battery voltage via ADC + voltage divider
//    - EMA filter for stable reading
//    - Color status: Green / Yellow / Red based on thresholds
// ═══════════════════════════════════════════════════════════════════════════════
#ifndef HAL_BATTERY_H
#define HAL_BATTERY_H

#include <stdint.h>
#include <stdbool.h>

// Battery status levels
#define BATT_LEVEL_CRITICAL     0   // < BATT_CRITICAL_MV (Red)
#define BATT_LEVEL_LOW          1   // < BATT_LOW_MV (Yellow)
#define BATT_LEVEL_NOMINAL      2   // < BATT_FULL_MV (Green)
#define BATT_LEVEL_FULL         3   // >= BATT_FULL_MV (Green)

void     hal_battery_init(void);

// Single instantaneous read (mV). ~50µs.
uint16_t hal_battery_read_mV(void);

// Filtered (EMA) voltage (mV). Call update periodically, then get result.
void     hal_battery_update(void);       // call at ~10-100 Hz
uint16_t hal_battery_get_avg_mV(void);   // last filtered value

// Status level (0=critical, 1=low, 2=nominal, 3=full)
uint8_t  hal_battery_get_level(void);

// Is battery critically low? (should stop motors)
bool     hal_battery_is_critical(void);

#endif // HAL_BATTERY_H
