// ═══════════════════════════════════════════════════════════════════════════════
//  pid.cpp — Generic PD/PID Controller Implementation
//
//  Discrete per-tick form (1kHz assumed):
//    P = Kp × error
//    I = Ki × Σ(error)         (with anti-windup)
//    D = Kd × (error - prev)   (per-tick delta)
// ═══════════════════════════════════════════════════════════════════════════════
#include "pid.h"
#include <math.h>

static inline float clampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

void pid_init(PidController* p, float Kp, float Ki, float Kd, float max_output) {
    p->Kp = Kp;
    p->Ki = Ki;
    p->Kd = Kd;
    p->max_output = max_output;
    p->prev_error = 0;
    p->integral   = 0;
    p->output     = 0;
}

void pid_set_gains(PidController* p, float Kp, float Ki, float Kd, float max_output) {
    p->Kp = Kp;
    p->Ki = Ki;
    p->Kd = Kd;
    p->max_output = max_output;
}

void pid_reset(PidController* p) {
    p->prev_error = 0;
    p->integral   = 0;
    p->output     = 0;
}

float pid_tick(PidController* p, float error) {
    // Proportional
    float pTerm = p->Kp * error;

    // Integral (with anti-windup: clamp integrator)
    float iTerm = 0;
    if (p->Ki > 0.0001f) {
        p->integral += error;
        // Anti-windup: limit integrator so Ki*integral doesn't exceed max
        float i_limit = p->max_output / p->Ki;
        p->integral = clampf(p->integral, -i_limit, i_limit);
        iTerm = p->Ki * p->integral;
    }

    // Derivative (on error, per-tick)
    float dTerm = p->Kd * (error - p->prev_error);
    p->prev_error = error;

    // Sum and clamp
    float out = pTerm + iTerm + dTerm;
    out = clampf(out, -p->max_output, p->max_output);
    p->output = out;
    return out;
}

float pid_get_output(const PidController* p) {
    return p->output;
}
