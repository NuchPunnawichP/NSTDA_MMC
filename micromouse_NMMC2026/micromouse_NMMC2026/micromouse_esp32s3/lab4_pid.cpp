// ═══════════════════════════════════════════════════════════════════════════════
//  lab4_pid.cpp — Lab 4: PID-Controlled Motion Implementation
//
//  Control architecture:
//    STRAIGHT:
//      vel_err    = target_vel − actual_vel   → vel_pid (both wheels)
//      heading_err = target_heading − actual   → heading_pid (differential)
//      vL = FF_L(target) + vel_pid − heading_pid
//      vR = FF_R(target) + vel_pid + heading_pid
//
//    TURN:
//      target_heading = ∫(profile angular velocity)
//      actual_heading  = ∫(gyro_z)
//      heading_err → heading_pid → differential drive
//      + FF base from angular velocity → wheel linear velocity
// ═══════════════════════════════════════════════════════════════════════════════
#include "lab4_pid.h"
#include "pid.h"
#include "motion_profile.h"
#include "feedforward.h"
#include "config.h"
#include <string.h>
#include <math.h>

// ── Helpers ─────────────────────────────────────────────────────────────────
static inline int16_t rd_i16(const uint8_t* p) {
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}
static inline int16_t clamp_i16(float v, int16_t lo, int16_t hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return (int16_t)v;
}

// ═══════════════════════════════════════════════════════════════════════════════
// State
// ═══════════════════════════════════════════════════════════════════════════════
enum Lab4Mode {
    L4_IDLE = 0,
    L4_STRAIGHT,
    L4_TURN,
};

static Lab4Mode     s_mode = L4_IDLE;
static MotionProfile s_profile;

// PID controllers
static PidController s_vel_pid;       // velocity: error in mm/s → output in mV
static PidController s_heading_pid;   // heading:  error in °     → output in mV

// Heading tracking
static float s_target_heading = 0;    // degrees (integrated from profile or held)
static float s_actual_heading  = 0;   // degrees (integrated from gyro)

// Distance tracking (for profile decel trigger)
static float s_actual_dist = 0;       // mm or degrees traveled
static float s_enc_accum   = 0;       // encoder accumulator

// Turn helper
static float s_turn_enc_L = 0;
static float s_turn_enc_R = 0;

// Safety timeout: max ticks before auto-stop (prevent runaway)
static uint32_t s_tick_count = 0;
#define LAB4_MAX_TICKS      10000   // 10 seconds max

// Steering trim: constant mV offset (+ = push right = fix left drift)
static int16_t s_steer_trim_mV = FF_STEERING_TRIM_MV;

#define DT 0.001f   // 1ms tick

// ═══════════════════════════════════════════════════════════════════════════════
// Init PIDs from config defaults
// ═══════════════════════════════════════════════════════════════════════════════
static bool s_pid_inited = false;

static void ensure_pid_init(void) {
    if (s_pid_inited) return;
    s_pid_inited = true;

    // Velocity PID: Kp in mV/(mm/s), Kd per-tick
    pid_init(&s_vel_pid,
             PD_LINEAR_KP_X100 / 100.0f,
             0,  // no Ki initially
             PD_LINEAR_KD_X100 / 100.0f,
             PD_LINEAR_MAX_MV);

    // Heading PID: Kp in mV/°, Ki for steady-state bias, Kd per-tick
    pid_init(&s_heading_pid,
             PD_STEER_KP_X100 / 100.0f,
             PD_STEER_KI_X100 / 100.0f,   // Ki: eliminates constant drift
             PD_STEER_KD_X100 / 100.0f,
             PD_STEER_MAX_MV);
}

// ═══════════════════════════════════════════════════════════════════════════════
// STRAIGHT handler
// ═══════════════════════════════════════════════════════════════════════════════
//  payload: [0]=test_id, [1..2]=distance_mm(i16), [3..4]=vel(i16), [5..6]=accel(i16)

static void handle_straight(const uint8_t* payload, CommandResult* res,
                             int16_t* vL, int16_t* vR) {
    int16_t dist  = rd_i16(&payload[1]);
    int16_t vel   = rd_i16(&payload[3]);
    int16_t accel = rd_i16(&payload[5]);

    if (vel <= 0) vel = SEARCH_VEL_MMPS;
    if (accel <= 0) accel = SEARCH_ACCEL_MMPS2;

    ensure_pid_init();
    pid_reset(&s_vel_pid);
    pid_reset(&s_heading_pid);

    s_mode = L4_STRAIGHT;
    s_actual_heading = 0;      // reset heading reference
    s_target_heading = 0;      // hold straight
    s_actual_dist = 0;
    s_enc_accum = 0;
    s_tick_count = 0;

    profile_plan_linear(&s_profile, (float)dist, (float)vel, (float)accel, (float)accel);

    *vL = 0; *vR = 0;
    res->status = RSP_OK;
}

// ═══════════════════════════════════════════════════════════════════════════════
// TURN handler
// ═══════════════════════════════════════════════════════════════════════════════
//  For TURN_90:  payload [1..2] = direction (+1 CW, -1 CCW)
//  For TURN_180: same

static void handle_turn(const uint8_t* payload, CommandResult* res,
                         int16_t* vL, int16_t* vR, float angle_deg) {
    int16_t dir = rd_i16(&payload[1]);
    float target_angle = angle_deg * ((dir >= 0) ? 1.0f : -1.0f);

    ensure_pid_init();
    pid_reset(&s_vel_pid);
    pid_reset(&s_heading_pid);

    // Use rotation PID gains for turn (tighter control)
    pid_set_gains(&s_heading_pid,
                  PD_ROT_KP_X100 / 100.0f,
                  0,
                  PD_ROT_KD_X100 / 100.0f,
                  PD_ROT_MAX_MV);

    s_mode = L4_TURN;
    s_actual_heading = 0;
    s_target_heading = 0;      // will be updated by profile integration
    s_actual_dist = 0;
    s_turn_enc_L = 0;
    s_turn_enc_R = 0;
    s_tick_count = 0;

    profile_plan_turn(&s_profile, target_angle,
                      (float)TURN_VEL_DPS, (float)TURN_ACCEL_DPS2, (float)TURN_ACCEL_DPS2);

    *vL = 0; *vR = 0;
    res->status = RSP_OK;
}

// ═══════════════════════════════════════════════════════════════════════════════
// MOVE_CELLS handler — straight N × CELL_SIZE_MM
// ═══════════════════════════════════════════════════════════════════════════════
static void handle_move_cells(const uint8_t* payload, CommandResult* res,
                               int16_t* vL, int16_t* vR) {
    int16_t n_cells = rd_i16(&payload[1]);
    int16_t vel     = rd_i16(&payload[3]);
    if (vel <= 0) vel = SEARCH_VEL_MMPS;
    float dist = (float)n_cells * CELL_SIZE_MM;

    // Reuse straight handler logic
    ensure_pid_init();
    pid_reset(&s_vel_pid);
    pid_reset(&s_heading_pid);

    s_mode = L4_STRAIGHT;
    s_actual_heading = 0;
    s_target_heading = 0;
    s_actual_dist = 0;
    s_enc_accum = 0;
    s_tick_count = 0;

    profile_plan_linear(&s_profile, dist, (float)vel,
                        (float)SEARCH_ACCEL_MMPS2, (float)SEARCH_DECEL_MMPS2);

    *vL = 0; *vR = 0;
    res->status = RSP_OK;
}

// ═══════════════════════════════════════════════════════════════════════════════
// Public API
// ═══════════════════════════════════════════════════════════════════════════════

bool lab4_handle_test(const CommandRequest* cmd, CommandResult* result,
                      int16_t* vCmdL_mV, int16_t* vCmdR_mV) {
    uint8_t test_id = cmd->payload[0];

    memset(result, 0, sizeof(*result));
    result->opcode = cmd->opcode;
    result->req_id = cmd->req_id;
    result->status = RSP_OK;

    switch (test_id) {
    case TEST_STRAIGHT:
        handle_straight(cmd->payload, result, vCmdL_mV, vCmdR_mV);
        return true;
    case TEST_TURN_90:
        handle_turn(cmd->payload, result, vCmdL_mV, vCmdR_mV, 90.0f);
        return true;
    case TEST_TURN_180:
        handle_turn(cmd->payload, result, vCmdL_mV, vCmdR_mV, 180.0f);
        return true;
    case TEST_MOVE_CELLS:
        handle_move_cells(cmd->payload, result, vCmdL_mV, vCmdR_mV);
        return true;
    default:
        return false;
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
// Tick @ 1kHz
// ═══════════════════════════════════════════════════════════════════════════════

bool lab4_tick(int16_t* vCmdL_mV, int16_t* vCmdR_mV,
               int16_t velL_mmps, int16_t velR_mmps,
               float gyroZ_dps) {

    if (s_mode == L4_IDLE) {
        *vCmdL_mV = 0; *vCmdR_mV = 0;
        return false;
    }

    // ── Safety timeout ──
    s_tick_count++;
    if (s_tick_count > LAB4_MAX_TICKS) {
        s_mode = L4_IDLE;
        *vCmdL_mV = 0; *vCmdR_mV = 0;
        return false;
    }

    // ── Integrate gyro for actual heading ──
    // GYRO_Z_SIGN flips sign so CW rotation = positive heading
    s_actual_heading += gyroZ_dps * (float)GYRO_Z_SIGN * DT;   // degrees

    if (s_mode == L4_STRAIGHT) {
        // ── Actual distance (mm) from average velocity integration ──
        float avg_vel = (float)(velL_mmps + velR_mmps) * 0.5f;
        s_actual_dist += fabsf(avg_vel) * DT;

        // ── Run profile → target velocity ──
        float target_vel = profile_tick(&s_profile, s_actual_dist);
        float target_acc = profile_get_accel(&s_profile);

        if (profile_is_done(&s_profile)) {
            s_mode = L4_IDLE;
            *vCmdL_mV = 0; *vCmdR_mV = 0;
            return false;
        }

        // ── Velocity PID ──
        float actual_vel = (float)(velL_mmps + velR_mmps) * 0.5f;
        float vel_err = target_vel - actual_vel;   // signed
        float vel_correction = pid_tick(&s_vel_pid, vel_err);

        // ── Heading PID (keep straight) ──
        // target_heading stays 0 (straight line)
        float heading_err = s_target_heading - s_actual_heading;
        float steer_correction = pid_tick(&s_heading_pid, heading_err);

        // ── Combine: FF + vel_PID ± heading_PID ± trim ──
        float ff_L = (float)ff_voltage_L(target_vel, target_acc);
        float ff_R = (float)ff_voltage_R(target_vel, target_acc);

        // Steering trim: + = add to left motor (fix left drift)
        *vCmdL_mV = clamp_i16(ff_L + vel_correction - steer_correction + s_steer_trim_mV, -VCMD_MAX_MV, VCMD_MAX_MV);
        *vCmdR_mV = clamp_i16(ff_R + vel_correction + steer_correction - s_steer_trim_mV, -VCMD_MAX_MV, VCMD_MAX_MV);

    } else if (s_mode == L4_TURN) {
        // ── Actual angle from gyro ──
        s_actual_dist = fabsf(s_actual_heading);   // degrees traveled

        // ── Run profile → target angular velocity (°/s) ──
        float target_omega = profile_tick(&s_profile, s_actual_dist);
        float target_alpha = profile_get_accel(&s_profile);

        // ── Integrate profile target to get target heading ──
        s_target_heading += target_omega * DT;

        if (profile_is_done(&s_profile)) {
            s_mode = L4_IDLE;
            *vCmdL_mV = 0; *vCmdR_mV = 0;

            // Restore steering PID gains (in case changed for turn)
            pid_set_gains(&s_heading_pid,
                          PD_STEER_KP_X100 / 100.0f, 0,
                          PD_STEER_KD_X100 / 100.0f,
                          PD_STEER_MAX_MV);
            return false;
        }

        // ── Heading PID ──
        float heading_err = s_target_heading - s_actual_heading;
        float heading_correction = pid_tick(&s_heading_pid, heading_err);

        // ── Feedforward: convert angular velocity to wheel linear velocity ──
        float omega_rad = target_omega * (3.14159265f / 180.0f);
        float v_wheel = omega_rad * WHEEL_BASE_MM * 0.5f;   // mm/s per wheel
        float a_wheel = target_alpha * (3.14159265f / 180.0f) * WHEEL_BASE_MM * 0.5f;

        // CW (+angle): left forward, right backward
        float ff_L = (float)ff_voltage_L( v_wheel,  a_wheel);
        float ff_R = (float)ff_voltage_R(-v_wheel, -a_wheel);

        // heading_correction: positive = need more CW = more left, less right
        *vCmdL_mV = clamp_i16(ff_L + heading_correction, -VCMD_MAX_MV, VCMD_MAX_MV);
        *vCmdR_mV = clamp_i16(ff_R - heading_correction, -VCMD_MAX_MV, VCMD_MAX_MV);
    }

    return true;
}

void lab4_stop(void) {
    s_mode = L4_IDLE;
    profile_reset(&s_profile);
}

bool lab4_is_running(void) {
    return s_mode != L4_IDLE;
}

// ── Runtime tuning ──────────────────────────────────────────────────────────

void lab4_set_vel_gains(float Kp, float Kd, float max_mV) {
    pid_set_gains(&s_vel_pid, Kp, 0, Kd, max_mV);
}

void lab4_set_heading_gains(float Kp, float Ki, float Kd, float max_mV) {
    pid_set_gains(&s_heading_pid, Kp, Ki, Kd, max_mV);
}

void lab4_set_steer_trim(int16_t trim_mV) {
    s_steer_trim_mV = trim_mV;
}
