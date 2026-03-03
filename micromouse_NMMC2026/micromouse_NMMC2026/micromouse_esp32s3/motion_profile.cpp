// ═══════════════════════════════════════════════════════════════════════════════
//  motion_profile.cpp — Trapezoidal Velocity Profile Implementation
// ═══════════════════════════════════════════════════════════════════════════════
#include "motion_profile.h"
#include <math.h>
#include <string.h>

#define DT  0.001f   // 1ms tick

// ═══════════════════════════════════════════════════════════════════════════════
// Internal: compute plan distances and peak velocity
// ═══════════════════════════════════════════════════════════════════════════════
static void compute_plan(MotionProfile* p) {
    float d  = p->total_dist;   // always positive
    float vm = p->max_vel;
    float a  = p->accel;
    float dc = p->decel;

    // Distance needed to accelerate to max_vel
    float d_accel = (vm * vm) / (2.0f * a);
    // Distance needed to decelerate from max_vel
    float d_decel = (vm * vm) / (2.0f * dc);

    if (d_accel + d_decel > d) {
        // Triangle profile: can't reach max_vel
        // v_peak = sqrt(2 * a * dc * d / (a + dc))
        float v_peak = sqrtf(2.0f * a * dc * d / (a + dc));
        p->vel_peak  = v_peak;
        p->dist_accel = (v_peak * v_peak) / (2.0f * a);
        p->dist_decel = d - p->dist_accel;
    } else {
        // Trapezoid: full accel, cruise, decel
        p->vel_peak  = vm;
        p->dist_accel = d_accel;
        p->dist_decel = d_decel;
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
// Plan
// ═══════════════════════════════════════════════════════════════════════════════

void profile_plan_linear(MotionProfile* p,
                         float distance_mm,
                         float max_vel_mmps,
                         float accel_mmps2,
                         float decel_mmps2) {
    memset(p, 0, sizeof(*p));

    if (fabsf(distance_mm) < 0.5f) {
        p->phase = PROF_DONE;
        return;
    }

    p->direction  = (distance_mm >= 0) ? 1 : -1;
    p->total_dist = fabsf(distance_mm);
    p->max_vel    = fabsf(max_vel_mmps);
    p->accel      = fabsf(accel_mmps2);
    p->decel      = (fabsf(decel_mmps2) > 0.1f) ? fabsf(decel_mmps2) : p->accel;

    compute_plan(p);

    p->phase        = PROF_ACCEL;
    p->target_vel   = 0;
    p->target_accel = p->accel;
    p->target_dist  = 0;
    p->tick_count   = 0;
}

void profile_plan_turn(MotionProfile* p,
                       float angle_deg,
                       float max_vel_dps,
                       float accel_dps2,
                       float decel_dps2) {
    // Same math, just units are degrees instead of mm
    profile_plan_linear(p, angle_deg, max_vel_dps, accel_dps2, decel_dps2);
}

// ═══════════════════════════════════════════════════════════════════════════════
// Tick — call at 1kHz
// ═══════════════════════════════════════════════════════════════════════════════

float profile_tick(MotionProfile* p, float actual_dist) {
    if (p->phase == PROF_IDLE || p->phase == PROF_DONE) {
        p->target_vel = 0;
        p->target_accel = 0;
        return 0;
    }

    p->tick_count++;

    switch (p->phase) {

    case PROF_ACCEL:
        p->target_accel = p->accel;
        p->target_vel  += p->accel * DT;

        if (p->target_vel >= p->vel_peak) {
            p->target_vel = p->vel_peak;

            // Check if there's a cruise phase
            float d_remaining = p->total_dist - p->dist_accel;
            if (d_remaining > p->dist_decel + 0.5f) {
                p->phase = PROF_CRUISE;
            } else {
                p->phase = PROF_DECEL;   // triangle: skip cruise
            }
        }
        break;

    case PROF_CRUISE: {
        p->target_accel = 0;
        p->target_vel   = p->vel_peak;

        // Start decelerating when remaining distance ≤ decel distance
        // Use actual encoder distance for accuracy
        float remain = p->total_dist - actual_dist;
        float d_stop = (p->target_vel * p->target_vel) / (2.0f * p->decel);

        if (remain <= d_stop + 1.0f) {
            p->phase = PROF_DECEL;
        }
        break;
    }

    case PROF_DECEL:
        p->target_accel = -p->decel;
        p->target_vel  -= p->decel * DT;

        if (p->target_vel <= 0 || actual_dist >= p->total_dist - 0.5f) {
            p->target_vel   = 0;
            p->target_accel = 0;
            p->phase = PROF_DONE;
        }
        break;

    default:
        break;
    }

    // Integrate target distance (for reference, but we prefer actual_dist)
    p->target_dist += p->target_vel * DT;

    // Return signed velocity
    return p->target_vel * p->direction;
}

// ═══════════════════════════════════════════════════════════════════════════════
// Queries
// ═══════════════════════════════════════════════════════════════════════════════

bool  profile_is_done(const MotionProfile* p)  { return p->phase == PROF_DONE; }
float profile_get_vel(const MotionProfile* p)   { return p->target_vel * p->direction; }
float profile_get_accel(const MotionProfile* p) { return p->target_accel * p->direction; }
float profile_get_dist(const MotionProfile* p)  { return p->target_dist; }
ProfilePhase profile_get_phase(const MotionProfile* p) { return p->phase; }

void profile_reset(MotionProfile* p) {
    memset(p, 0, sizeof(*p));
    p->phase = PROF_IDLE;
}
