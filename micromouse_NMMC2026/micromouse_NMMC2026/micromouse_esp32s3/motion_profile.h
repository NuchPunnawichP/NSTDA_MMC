// ═══════════════════════════════════════════════════════════════════════════════
//  motion_profile.h — Trapezoidal Velocity Profile Generator
//
//  Layer: Control (used by Lab 3+ test handlers)
//
//  Design:
//    Time-based trapezoid with position-aware deceleration trigger.
//    Each tick (1ms): updates target velocity, tracks distance.
//    Uses encoder feedback to decide when to start decelerating.
//
//  Usage:
//    1. profile_plan_linear(&p, distance_mm, max_vel, accel, decel)
//    2. Each 1kHz tick: profile_tick(&p, actual_dist_mm) → target vel/accel
//    3. Apply feedforward: vCmd = ff(target_vel, target_accel)
// ═══════════════════════════════════════════════════════════════════════════════
#ifndef MOTION_PROFILE_H
#define MOTION_PROFILE_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// ── Profile phases ─────────────────────────────────────────────────────────
typedef enum {
    PROF_IDLE = 0,
    PROF_ACCEL,
    PROF_CRUISE,
    PROF_DECEL,
    PROF_DONE,
} ProfilePhase;

// ── Profile state ──────────────────────────────────────────────────────────
typedef struct {
    // Plan (set once)
    float total_dist;       // target distance (mm or degrees)
    float max_vel;          // peak velocity (mm/s or °/s)
    float accel;            // acceleration rate
    float decel;            // deceleration rate
    float dist_accel;       // distance to accelerate from 0 to max_vel
    float dist_decel;       // distance to decelerate from max_vel to 0
    float vel_peak;         // actual peak vel (may be < max_vel if triangle)
    int8_t direction;       // +1 or -1

    // Runtime (updated each tick)
    ProfilePhase phase;
    float target_vel;       // current target velocity
    float target_accel;     // current target acceleration
    float target_dist;      // distance traveled by profile (integrated target_vel)
    uint32_t tick_count;    // elapsed ticks
} MotionProfile;

// ── Plan a linear (forward/backward) profile ──────────────────────────────
//  distance_mm > 0 = forward, < 0 = backward
void profile_plan_linear(MotionProfile* p,
                         float distance_mm,
                         float max_vel_mmps,
                         float accel_mmps2,
                         float decel_mmps2);

// ── Plan a rotation profile ───────────────────────────────────────────────
//  angle_deg > 0 = CW, < 0 = CCW
void profile_plan_turn(MotionProfile* p,
                       float angle_deg,
                       float max_vel_dps,
                       float accel_dps2,
                       float decel_dps2);

// ── Tick (call @ 1kHz) ────────────────────────────────────────────────────
//  actual_dist = real distance traveled (from encoders/gyro, always positive)
//  Returns target velocity (signed). Also sets p->target_accel.
float profile_tick(MotionProfile* p, float actual_dist);

// ── Query ──────────────────────────────────────────────────────────────────
bool  profile_is_done(const MotionProfile* p);
float profile_get_vel(const MotionProfile* p);
float profile_get_accel(const MotionProfile* p);
float profile_get_dist(const MotionProfile* p);
ProfilePhase profile_get_phase(const MotionProfile* p);

// ── Reset / Abort ──────────────────────────────────────────────────────────
void profile_reset(MotionProfile* p);

#ifdef __cplusplus
}
#endif

#endif // MOTION_PROFILE_H
