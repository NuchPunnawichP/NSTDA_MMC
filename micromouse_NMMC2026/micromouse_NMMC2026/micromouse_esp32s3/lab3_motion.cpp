// ═══════════════════════════════════════════════════════════════════════════════
//  lab3_motion.cpp — Lab 3: Motion Profile Implementation
//
//  Straight: both wheels same direction, track distance via encoder average
//  Turn:     differential drive, track angle via encoder differential
//  FF:       per-wheel feedforward using Lab 2b measured parameters
// ═══════════════════════════════════════════════════════════════════════════════
#include "lab3_motion.h"
#include "motion_profile.h"
#include "feedforward.h"
#include "config.h"
#include "hal_encoder.h"
#include <string.h>
#include <math.h>

// ── Helpers ─────────────────────────────────────────────────────────────────
static inline int16_t rd_i16(const uint8_t* p) {
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}
static inline uint16_t rd_u16(const uint8_t* p) {
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

// ── State ───────────────────────────────────────────────────────────────────
enum Lab3Mode {
    L3_IDLE = 0,
    L3_STRAIGHT,
    L3_TURN,
};

static Lab3Mode     s_mode = L3_IDLE;
static MotionProfile s_profile;

// Accumulated distance from encoder (mm or degrees)
static float s_actual_dist  = 0;

// Encoder accumulator (counts)
static int32_t s_enc_accum_L = 0;
static int32_t s_enc_accum_R = 0;

// ═══════════════════════════════════════════════════════════════════════════════
// STRAIGHT: dist_mm = (encL + encR) / 2 × MM_PER_COUNT
// ═══════════════════════════════════════════════════════════════════════════════

static void handle_trap_fwd(const uint8_t* payload, CommandResult* res,
                             int16_t* vL, int16_t* vR) {
    int16_t  dist_mm  = rd_i16(&payload[1]);    // signed: +fwd, -back
    int16_t  max_vel  = rd_i16(&payload[3]);    // mm/s (unsigned intended)
    int16_t  accel    = rd_i16(&payload[5]);    // mm/s²

    if (max_vel <= 0) max_vel = SEARCH_VEL_MMPS;
    if (accel <= 0)   accel = SEARCH_ACCEL_MMPS2;

    s_mode = L3_STRAIGHT;
    s_actual_dist = 0;
    s_enc_accum_L = 0;
    s_enc_accum_R = 0;

    profile_plan_linear(&s_profile,
                        (float)dist_mm,
                        (float)max_vel,
                        (float)accel,
                        (float)accel);      // decel = accel (symmetric)

    *vL = 0; *vR = 0;
    res->status = RSP_OK;
}

// ═══════════════════════════════════════════════════════════════════════════════
// TURN: angle_deg = (encR - encL) × MM_PER_COUNT / WHEEL_BASE × (180/π)
// ═══════════════════════════════════════════════════════════════════════════════

static void handle_trap_turn(const uint8_t* payload, CommandResult* res,
                              int16_t* vL, int16_t* vR) {
    int16_t  angle_deg = rd_i16(&payload[1]);   // signed: +CW, -CCW
    int16_t  max_dps   = rd_i16(&payload[3]);   // °/s
    int16_t  accel_dps = rd_i16(&payload[5]);   // °/s²

    if (max_dps <= 0)   max_dps = TURN_VEL_DPS;
    if (accel_dps <= 0) accel_dps = TURN_ACCEL_DPS2;

    s_mode = L3_TURN;
    s_actual_dist = 0;
    s_enc_accum_L = 0;
    s_enc_accum_R = 0;

    profile_plan_turn(&s_profile,
                      (float)angle_deg,
                      (float)max_dps,
                      (float)accel_dps,
                      (float)accel_dps);

    *vL = 0; *vR = 0;
    res->status = RSP_OK;
}

// ═══════════════════════════════════════════════════════════════════════════════
// Public API
// ═══════════════════════════════════════════════════════════════════════════════

bool lab3_handle_test(const CommandRequest* cmd, CommandResult* result,
                      int16_t* vCmdL_mV, int16_t* vCmdR_mV) {
    uint8_t test_id = cmd->payload[0];

    memset(result, 0, sizeof(*result));
    result->opcode = cmd->opcode;
    result->req_id = cmd->req_id;
    result->status = RSP_OK;

    switch (test_id) {
    case TEST_TRAP_FWD:
        handle_trap_fwd(cmd->payload, result, vCmdL_mV, vCmdR_mV);
        return true;
    case TEST_TRAP_TURN:
        handle_trap_turn(cmd->payload, result, vCmdL_mV, vCmdR_mV);
        return true;
    default:
        return false;
    }
}

bool lab3_tick(int16_t* vCmdL_mV, int16_t* vCmdR_mV,
               int16_t encDeltaL, int16_t encDeltaR) {

    if (s_mode == L3_IDLE) {
        *vCmdL_mV = 0; *vCmdR_mV = 0;
        return false;
    }

    // Accumulate encoder counts
    s_enc_accum_L += encDeltaL;
    s_enc_accum_R += encDeltaR;

    // ── Compute actual distance ──
    if (s_mode == L3_STRAIGHT) {
        // Average of both wheels (mm)
        float avg_counts = (float)(s_enc_accum_L + s_enc_accum_R) * 0.5f;
        s_actual_dist = fabsf(avg_counts * MM_PER_COUNT);
    } else {
        // Turn: angle from encoder differential
        // Δangle (rad) = (right - left) × MM_PER_COUNT / WHEEL_BASE
        // Δangle (deg) = rad × 180/π
        float diff_mm = (float)(s_enc_accum_R - s_enc_accum_L) * MM_PER_COUNT;
        s_actual_dist = fabsf(diff_mm / WHEEL_BASE_MM * (180.0f / 3.14159265f));
    }

    // ── Run profile ──
    float target_vel = profile_tick(&s_profile, s_actual_dist);
    float target_acc = profile_get_accel(&s_profile);

    // ── Check done ──
    if (profile_is_done(&s_profile)) {
        s_mode = L3_IDLE;
        *vCmdL_mV = 0; *vCmdR_mV = 0;
        return false;
    }

    // ── Apply feedforward ──
    if (s_mode == L3_STRAIGHT) {
        // Both wheels same velocity
        *vCmdL_mV = ff_voltage_L(target_vel, target_acc);
        *vCmdR_mV = ff_voltage_R(target_vel, target_acc);
    } else {
        // Turn: left and right opposite
        // CW turn (+angle): left forward, right backward
        // Convert angular vel (°/s) to wheel linear vel (mm/s)
        // v_wheel = ω × WHEEL_BASE/2, where ω in rad/s
        float omega_rad = target_vel * (3.14159265f / 180.0f);
        float v_wheel = omega_rad * WHEEL_BASE_MM * 0.5f;  // mm/s per wheel
        float a_wheel = target_acc * (3.14159265f / 180.0f) * WHEEL_BASE_MM * 0.5f;

        // CW: left forward (+), right backward (-)
        *vCmdL_mV =  ff_voltage_L( v_wheel,  a_wheel);
        *vCmdR_mV =  ff_voltage_R(-v_wheel, -a_wheel);
    }

    return true;
}

void lab3_stop(void) {
    s_mode = L3_IDLE;
    profile_reset(&s_profile);
}

bool lab3_is_running(void) {
    return s_mode != L3_IDLE;
}
