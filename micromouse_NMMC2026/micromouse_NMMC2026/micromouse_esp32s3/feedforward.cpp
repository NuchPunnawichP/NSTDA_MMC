// ═══════════════════════════════════════════════════════════════════════════════
//  feedforward.cpp — Feedforward Motor Voltage Calculator (per-wheel slope + signed accel + start-boost)
// ═══════════════════════════════════════════════════════════════════════════════
#include "feedforward.h"
#include "config.h"
#include <math.h>

// ───────────────────────────────────────────────────────────────────────────────
// Fallbacks (so it still compiles even if some macros aren't defined yet)
// ───────────────────────────────────────────────────────────────────────────────

// Per-wheel kV (mV per (mm/s)) ×100
#ifndef FF_LINEAR_SPEED_L_X100
  #define FF_LINEAR_SPEED_L_X100 FF_LINEAR_SPEED_X100
#endif
#ifndef FF_LINEAR_SPEED_R_X100
  #define FF_LINEAR_SPEED_R_X100 FF_LINEAR_SPEED_X100
#endif

// "Start" bias (stiction) — what you measured in FIND_BIAS
#ifndef FF_LINEAR_BIAS_L_MV
  #define FF_LINEAR_BIAS_L_MV 0
#endif
#ifndef FF_LINEAR_BIAS_R_MV
  #define FF_LINEAR_BIAS_R_MV 0
#endif

// "Run" bias (dynamic intercept) — recommended to add in config.h
#ifndef FF_LINEAR_RUN_BIAS_L_MV
  #define FF_LINEAR_RUN_BIAS_L_MV FF_LINEAR_BIAS_L_MV
#endif
#ifndef FF_LINEAR_RUN_BIAS_R_MV
  #define FF_LINEAR_RUN_BIAS_R_MV FF_LINEAR_BIAS_R_MV
#endif

// Accel FF (signed): mV per (mm/s²) ×100
#ifndef FF_LINEAR_ACC_X100
  #define FF_LINEAR_ACC_X100 0
#endif

// Blend region for start-boost (mm/s). Below V0 → mostly "start bias", above V1 → "run bias"
#ifndef FF_START_BLEND_V0_MMPS
  #define FF_START_BLEND_V0_MMPS 0.0f
#endif
#ifndef FF_START_BLEND_V1_MMPS
  #define FF_START_BLEND_V1_MMPS 120.0f
#endif

// ───────────────────────────────────────────────────────────────────────────────

static const float SPEED_FF_L = FF_LINEAR_SPEED_L_X100 / 100.0f;
static const float SPEED_FF_R = FF_LINEAR_SPEED_R_X100 / 100.0f;
static const float ACC_FF     = FF_LINEAR_ACC_X100     / 100.0f;

static inline int16_t clamp_mV(float v) {
    if (v >  VCMD_MAX_MV) return  VCMD_MAX_MV;
    if (v < -VCMD_MAX_MV) return -VCMD_MAX_MV;
    return (int16_t)lroundf(v);
}

static inline float sgnf_safe(float v, float a) {
    if (fabsf(v) >= 1.0f) return (v > 0) ? 1.0f : -1.0f;
    if (fabsf(a) >= 1.0f) return (a > 0) ? 1.0f : -1.0f;
    return 0.0f;
}

static inline float clamp01(float x) {
    if (x < 0.0f) return 0.0f;
    if (x > 1.0f) return 1.0f;
    return x;
}

static inline float ff_lerp(float a, float b, float t) {
    return a + (b - a) * t;
}

static inline float blended_bias(float v_abs, float bias_start, float bias_run) {
    // Smoothly transition from "start bias" → "run bias" as target speed increases
    float t = clamp01((v_abs - FF_START_BLEND_V0_MMPS) / (FF_START_BLEND_V1_MMPS - FF_START_BLEND_V0_MMPS));
    return ff_lerp(bias_start, bias_run, t);
}

static inline int16_t compute(float vel_mmps, float acc_mmps2,
                              float bias_start_mV, float bias_run_mV,
                              float kV_mV_per_mmps) {
    float sign = sgnf_safe(vel_mmps, acc_mmps2);
    if (sign == 0.0f) return 0;

    float v_abs = fabsf(vel_mmps);

    // Use dynamic "run" bias for accuracy, with a small "start boost" near zero speed.
    float bias = blended_bias(v_abs, bias_start_mV, bias_run_mV);

    // IMPORTANT: signed accel term (helps decel instead of fighting it)
    float cmd = sign * (bias + kV_mV_per_mmps * v_abs) + ACC_FF * acc_mmps2;

    return clamp_mV(cmd);
}

int16_t ff_voltage_L(float vel_mmps, float accel_mmps2) {
    return compute(vel_mmps, accel_mmps2,
                   (float)FF_LINEAR_BIAS_L_MV, (float)FF_LINEAR_RUN_BIAS_L_MV,
                   SPEED_FF_L);
}

int16_t ff_voltage_R(float vel_mmps, float accel_mmps2) {
    return compute(vel_mmps, accel_mmps2,
                   (float)FF_LINEAR_BIAS_R_MV, (float)FF_LINEAR_RUN_BIAS_R_MV,
                   SPEED_FF_R);
}

// Optional symmetric helper (uses averages)
int16_t ff_voltage(float vel_mmps, float accel_mmps2) {
    float bias_start = 0.5f * ((float)FF_LINEAR_BIAS_L_MV + (float)FF_LINEAR_BIAS_R_MV);
    float bias_run   = 0.5f * ((float)FF_LINEAR_RUN_BIAS_L_MV + (float)FF_LINEAR_RUN_BIAS_R_MV);
    float kV         = 0.5f * (SPEED_FF_L + SPEED_FF_R);

    return compute(vel_mmps, accel_mmps2, bias_start, bias_run, kV);
}
