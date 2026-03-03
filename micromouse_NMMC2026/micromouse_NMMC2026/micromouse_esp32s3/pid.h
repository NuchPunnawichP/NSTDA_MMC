// ═══════════════════════════════════════════════════════════════════════════════
//  pid.h — Generic PD/PID Controller
//
//  Design:
//    - PD by default (no integrator windup risk)
//    - Optional Ki with anti-windup clamp
//    - Gains stored as float for precision
//    - Output clamped to ±max_output
//    - Call pid_tick() at control rate (1kHz)
// ═══════════════════════════════════════════════════════════════════════════════
#ifndef PID_H
#define PID_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    // Gains
    float Kp;
    float Ki;
    float Kd;
    float max_output;       // ±clamp

    // State
    float prev_error;
    float integral;
    float output;
} PidController;

// Initialize PID with gains (Ki=0 for PD only)
void pid_init(PidController* p, float Kp, float Ki, float Kd, float max_output);

// Set gains at runtime (for tuning via BLE)
void pid_set_gains(PidController* p, float Kp, float Ki, float Kd, float max_output);

// Reset state (call when starting a new motion)
void pid_reset(PidController* p);

// Compute one tick. error = setpoint - measurement.
// dt is implicit (assumed constant 1ms = 0.001s)
// Returns clamped output.
float pid_tick(PidController* p, float error);

// Read last output
float pid_get_output(const PidController* p);

#ifdef __cplusplus
}
#endif

#endif // PID_H
