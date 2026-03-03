// ═══════════════════════════════════════════════════════════════════════════════
//  hal_wallsensor.h — Wall Sensor Manager (Abstraction Layer)
//
//  Layer: HAL
//  Dependencies: config.h, Wire1 (for VL6180X)
//
//  Design:
//    - Unified API regardless of sensor type (IR analog / VL6180X ToF / Mixed)
//    - Student configures type, count, and positions in config.h
//    - Upper layers call hal_wall_get_mm(position) — always returns mm
//    - Manager handles:
//        • VL6180X: I2C re-addressing, ranging, error handling
//        • IR analog: ADC read, lookup table / formula → mm conversion
//        • Mixed: per-slot type dispatch
//    - Output filtered by configurable method (raw / median3 / EMA)
//    - ★ I2C/ADC reads are slow — call hal_wall_update() from SensorTask
//      at ~30-40 Hz (NOT in step1kHz)
// ═══════════════════════════════════════════════════════════════════════════════
#ifndef HAL_WALLSENSOR_H
#define HAL_WALLSENSOR_H

#include <stdint.h>
#include <stdbool.h>
#include "protocol.h"   // for SNS_POS_* constants

// Number of sensor positions supported
#define WALL_POS_COUNT          SNS_POS_MAX     // 4 standard positions

// ── Initialization ──────────────────────────────────────────────────────────
// Initializes all configured sensors based on config.h settings.
// For VL6180X: starts I2C Bus 1, re-addresses each sensor via XSHUT.
// For IR: configures ADC pins.
// Returns true if at least one sensor is operational.
bool hal_wall_init(void);

// ── Update all sensors (call from SensorTask @ 30-40Hz) ─────────────────────
// Reads all active sensor slots, applies offset + clamp + filter.
// Call once per sensor cycle — NOT in step1kHz().
void hal_wall_update(void);

// ── Get distance at position (mm) ───────────────────────────────────────────
//  position: SNS_POS_LEFT, SNS_POS_FRONT_LEFT, SNS_POS_FRONT_RIGHT, SNS_POS_RIGHT
//  Returns: distance in mm (0 = no sensor / invalid)
uint16_t hal_wall_get_mm(uint8_t position);

// ── Get all distances at once ───────────────────────────────────────────────
//  out[0..3] = Left, FrontLeft, FrontRight, Right (mm)
void hal_wall_get_all(uint16_t out[WALL_POS_COUNT]);

// ── Wall detection (using thresholds from config.h) ─────────────────────────
bool hal_wall_has_left(void);
bool hal_wall_has_front_left(void);
bool hal_wall_has_front_right(void);
bool hal_wall_has_right(void);
bool hal_wall_has_front(void);  // true if either FL or FR detects wall

// ── Get wall bitmap (4 bits: L FL FR R) ─────────────────────────────────────
//  bit0 = left, bit1 = front-left, bit2 = front-right, bit3 = right
uint8_t hal_wall_bitmap(void);

// ── Get sensor type at position ─────────────────────────────────────────────
uint8_t hal_wall_get_type(uint8_t position);  // 0=none, 1=IR, 2=VL6180X

// ── Status ──────────────────────────────────────────────────────────────────
bool    hal_wall_is_init(void);
uint8_t hal_wall_active_count(void);    // how many sensors are online

// ── Calibration (measure-only, returns raw averages) ─────────────────────────
// Samples all sensors N times, averages readings (before offset).
// Does NOT modify offsets. Use hal_wall_set_offset() to apply computed offsets.
// Blocks for ~(samples * 25) ms. Typical: 50 samples = ~1.25s.
void hal_wall_calibrate(uint16_t samples);

// Set/get offset per sensor position (runtime, overrides config.h value)
void    hal_wall_set_offset(uint8_t position, int16_t offset_mm);
int16_t hal_wall_get_offset(uint8_t position);

// Get last calibration raw average per sensor (mm before offset)
uint16_t hal_wall_get_cal_raw(uint8_t position);

#endif // HAL_WALLSENSOR_H
