// ═══════════════════════════════════════════════════════════════════════════════
//  hal_encoder.h — Encoder HAL (PCNT x4 Quadrature Decoding)
//
//  Layer: HAL
//  Dependencies: config.h
//
//  Design:
//    - Uses ESP32-S3 PCNT hardware for zero-CPU-cost counting
//    - x4 decoding: counts both edges of both channels → max resolution
//    - Direction from config.h (ENC_L_DIRECTION, ENC_R_DIRECTION)
//    - Forward motion = positive counts
//    - Provides both absolute count and delta-per-call
//    - Delta is used by ControlTask @ 1kHz for velocity estimation
// ═══════════════════════════════════════════════════════════════════════════════
#ifndef HAL_ENCODER_H
#define HAL_ENCODER_H

#include <stdint.h>
#include <stdbool.h>

// ── Initialization ──────────────────────────────────────────────────────────
bool hal_encoder_init(void);

// ── Read absolute count (since last reset) ──────────────────────────────────
int32_t hal_encoder_count_L(void);
int32_t hal_encoder_count_R(void);

// ── Read delta (counts since last delta call) ───────────────────────────────
//    Call once per 1kHz tick. Returns count change since previous call.
//    Thread-safe: uses atomic snapshot of hardware counter.
int16_t hal_encoder_delta_L(void);
int16_t hal_encoder_delta_R(void);

// ── Reset counters to zero ──────────────────────────────────────────────────
void hal_encoder_reset(void);
void hal_encoder_reset_L(void);
void hal_encoder_reset_R(void);

// ── Status ──────────────────────────────────────────────────────────────────
bool hal_encoder_is_init(void);

#endif // HAL_ENCODER_H
