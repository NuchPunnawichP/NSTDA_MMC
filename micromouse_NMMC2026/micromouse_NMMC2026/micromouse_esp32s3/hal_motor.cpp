// ═══════════════════════════════════════════════════════════════════════════════
//  hal_motor.cpp — Motor HAL Implementation
// ═══════════════════════════════════════════════════════════════════════════════
#include "hal_motor.h"
#include "config.h"
#include <Arduino.h>

// ── State ───────────────────────────────────────────────────────────────────
static bool     s_init = false;
static int8_t   s_dutyL_pct = 0;   // last duty for telemetry (-100..+100)
static int8_t   s_dutyR_pct = 0;

// ── Internal: set single motor raw PWM ──────────────────────────────────────
//  duty: 0..MOTOR_PWM_MAX for magnitude
//  direction: +1 = forward, -1 = reverse, 0 = coast
static void motor_set_raw(uint8_t pin_in1, uint8_t pin_in2,
                           int16_t duty, int8_t dir) {
    if (dir > 0) {
        // Forward: IN1=PWM, IN2=0
        ledcWrite(pin_in1, (uint32_t)duty);
        ledcWrite(pin_in2, 0);
    } else if (dir < 0) {
        // Reverse: IN1=0, IN2=PWM
        ledcWrite(pin_in1, 0);
        ledcWrite(pin_in2, (uint32_t)duty);
    } else {
        // Coast: both LOW
        ledcWrite(pin_in1, 0);
        ledcWrite(pin_in2, 0);
    }
}

// ── Public API ──────────────────────────────────────────────────────────────

void hal_motor_init(void) {
    // Arduino ESP32 core 3.x: ledcAttach(pin, freq, resolution)
    // Each pin gets its own LEDC channel (auto-assigned)
    ledcAttach(PIN_MOTOR_L_IN1, MOTOR_PWM_FREQ, MOTOR_PWM_RESOLUTION);
    ledcAttach(PIN_MOTOR_L_IN2, MOTOR_PWM_FREQ, MOTOR_PWM_RESOLUTION);
    ledcAttach(PIN_MOTOR_R_IN1, MOTOR_PWM_FREQ, MOTOR_PWM_RESOLUTION);
    ledcAttach(PIN_MOTOR_R_IN2, MOTOR_PWM_FREQ, MOTOR_PWM_RESOLUTION);

    // Start in coast mode
    hal_motor_coast();
    s_init = true;
}

void hal_motor_set_mV(int16_t vCmdL_mV, int16_t vCmdR_mV, uint16_t vBat_mV) {
    if (!s_init) return;

    // Guard: vBat must be valid (avoid division by zero)
    if (vBat_mV < 3000) {
        // Battery too low or invalid → coast for safety
        hal_motor_coast();
        s_dutyL_pct = 0;
        s_dutyR_pct = 0;
        return;
    }

    // Clamp vCmd to max allowed
    if (vCmdL_mV >  VCMD_MAX_MV) vCmdL_mV =  VCMD_MAX_MV;
    if (vCmdL_mV < -VCMD_MAX_MV) vCmdL_mV = -VCMD_MAX_MV;
    if (vCmdR_mV >  VCMD_MAX_MV) vCmdR_mV =  VCMD_MAX_MV;
    if (vCmdR_mV < -VCMD_MAX_MV) vCmdR_mV = -VCMD_MAX_MV;

    // ── Left motor ──
    // Apply direction config
    int16_t cmdL = vCmdL_mV * MOTOR_L_DIRECTION;
    int8_t  dirL = (cmdL > 0) ? 1 : (cmdL < 0) ? -1 : 0;

    // duty = |vCmd_mV| / vBat_mV * PWM_MAX
    uint32_t absL = (cmdL >= 0) ? (uint32_t)cmdL : (uint32_t)(-cmdL);
    uint32_t dutyL = (absL * MOTOR_PWM_MAX) / vBat_mV;
    if (dutyL > MOTOR_PWM_MAX) dutyL = MOTOR_PWM_MAX;

    // Store duty percent for telemetry
    s_dutyL_pct = (int8_t)(((int32_t)dutyL * 100 * dirL) / MOTOR_PWM_MAX);

    motor_set_raw(PIN_MOTOR_L_IN1, PIN_MOTOR_L_IN2, (int16_t)dutyL, dirL);

    // ── Right motor ──
    int16_t cmdR = vCmdR_mV * MOTOR_R_DIRECTION;
    int8_t  dirR = (cmdR > 0) ? 1 : (cmdR < 0) ? -1 : 0;

    uint32_t absR = (cmdR >= 0) ? (uint32_t)cmdR : (uint32_t)(-cmdR);
    uint32_t dutyR = (absR * MOTOR_PWM_MAX) / vBat_mV;
    if (dutyR > MOTOR_PWM_MAX) dutyR = MOTOR_PWM_MAX;

    s_dutyR_pct = (int8_t)(((int32_t)dutyR * 100 * dirR) / MOTOR_PWM_MAX);

    motor_set_raw(PIN_MOTOR_R_IN1, PIN_MOTOR_R_IN2, (int16_t)dutyR, dirR);
}

void hal_motor_brake(void) {
    if (!s_init) return;
    // Short brake: both pins HIGH
    ledcWrite(PIN_MOTOR_L_IN1, MOTOR_PWM_MAX);
    ledcWrite(PIN_MOTOR_L_IN2, MOTOR_PWM_MAX);
    ledcWrite(PIN_MOTOR_R_IN1, MOTOR_PWM_MAX);
    ledcWrite(PIN_MOTOR_R_IN2, MOTOR_PWM_MAX);
    s_dutyL_pct = 0;
    s_dutyR_pct = 0;
}

void hal_motor_coast(void) {
    if (!s_init) return;
    // Coast: both pins LOW
    ledcWrite(PIN_MOTOR_L_IN1, 0);
    ledcWrite(PIN_MOTOR_L_IN2, 0);
    ledcWrite(PIN_MOTOR_R_IN1, 0);
    ledcWrite(PIN_MOTOR_R_IN2, 0);
    s_dutyL_pct = 0;
    s_dutyR_pct = 0;
}

bool hal_motor_is_init(void) {
    return s_init;
}

int8_t hal_motor_get_duty_pct_L(void) { return s_dutyL_pct; }
int8_t hal_motor_get_duty_pct_R(void) { return s_dutyR_pct; }
