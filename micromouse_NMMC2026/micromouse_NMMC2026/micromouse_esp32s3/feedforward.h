// ═══════════════════════════════════════════════════════════════════════════════
//  feedforward.h — Feedforward Motor Voltage Calculator
//
//  Model: vCmd_mV = sign(vel) × (bias + speed_ff × |vel| + acc_ff × |accel|)
//
//  Uses per-motor bias from Lab 2b measurements.
//  Parameters from config.h (FF_LINEAR_*).
// ═══════════════════════════════════════════════════════════════════════════════
#ifndef FEEDFORWARD_H
#define FEEDFORWARD_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Compute feedforward voltage for left motor
// vel_mmps: signed velocity (mm/s), accel: signed acceleration (mm/s²)
int16_t ff_voltage_L(float vel_mmps, float accel_mmps2);

// Compute feedforward voltage for right motor
int16_t ff_voltage_R(float vel_mmps, float accel_mmps2);

// Symmetric (average bias) — for differential turn drive
int16_t ff_voltage(float vel_mmps, float accel_mmps2);

#ifdef __cplusplus
}
#endif

#endif // FEEDFORWARD_H
