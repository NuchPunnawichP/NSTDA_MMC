// ═══════════════════════════════════════════════════════════════════════════════
//  hal_motor.h — Motor HAL (DRV8833 Dual H-Bridge)
//
//  Layer: HAL (lowest)
//  Dependencies: config.h only
//
//  Design:
//    - Input:  vCmd_mV (millivolt command) + vBat_mV (current battery voltage)
//    - Output: PWM duty cycle calculated internally: duty = vCmd_mV / vBat_mV
//    - Upper layers NEVER touch PWM directly (Iron Rule)
//    - Uses LEDC peripheral (Arduino ESP32 core 3.x API)
//    - DRV8833 control: IN1/IN2 per motor
//        Forward: IN1=PWM, IN2=0
//        Reverse: IN1=0,   IN2=PWM
//        Brake:   IN1=MAX, IN2=MAX (short brake)
//        Coast:   IN1=0,   IN2=0
// ═══════════════════════════════════════════════════════════════════════════════
#ifndef HAL_MOTOR_H
#define HAL_MOTOR_H

#include <stdint.h>
#include <stdbool.h>

// ── Initialization ──────────────────────────────────────────────────────────
// Call once in setup(). Configures LEDC channels for DRV8833.
void hal_motor_init(void);

// ── Motor Control (called from ControlTask @ 1kHz) ──────────────────────────
//
//  vCmdL_mV / vCmdR_mV: voltage command in millivolts
//    positive = forward, negative = reverse, 0 = coast
//  vBat_mV: current battery voltage in millivolts (must be > 0)
//
//  Internally computes: duty = abs(vCmd_mV) / vBat_mV * PWM_MAX
//  Clamps to MOTOR_PWM_MAX. Applies direction from config.h.
void hal_motor_set_mV(int16_t vCmdL_mV, int16_t vCmdR_mV, uint16_t vBat_mV);

// ── Safety Controls ─────────────────────────────────────────────────────────
// Brake: active braking (both pins HIGH → motor shorted)
void hal_motor_brake(void);

// Coast: free-running (both pins LOW)
void hal_motor_coast(void);

// ── Status ──────────────────────────────────────────────────────────────────
bool hal_motor_is_init(void);

// ── Debug: get last computed duty (for telemetry) ───────────────────────────
// Returns ±100 scale (-100..+100 percent), updated after each set_mV call
int8_t hal_motor_get_duty_pct_L(void);
int8_t hal_motor_get_duty_pct_R(void);

#endif // HAL_MOTOR_H
