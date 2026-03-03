// ═══════════════════════════════════════════════════════════════════════════════
//  lab6_speedrun.cpp — Lab 6: Speed Run through Known Maze
//
//  Algorithm:
//    1. Get maze from Lab 5 (lab5_get_maze)
//    2. Flood fill from goal → trace shortest path from start
//    3. Compress into segments: {direction, cell_count}
//    4. Execute: for each segment → drive N cells → turn → next
//
//  Speed levels (from config.h):
//    0: SPEEDRUN_VEL_0 (cautious, same as search)
//    1: SPEEDRUN_VEL_1 (moderate)
//    2: SPEEDRUN_VEL_2 (fast)
//    3: SPEEDRUN_VEL_3 (fastest)
// ═══════════════════════════════════════════════════════════════════════════════
#include "lab6_speedrun.h"
#include "lab5_search.h"
#include "maze.h"
#include "pid.h"
#include "motion_profile.h"
#include "feedforward.h"
#include "config.h"
#include <string.h>
#include <math.h>

// ── Helpers ──────────────────────────────────────────────────────────────────
static inline int16_t clamp_i16(float v, int16_t lo, int16_t hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return (int16_t)v;
}

#define DT 0.001f

// ═══════════════════════════════════════════════════════════════════════════════
// Path segment: drive N cells in direction D
// ═══════════════════════════════════════════════════════════════════════════════
typedef struct {
    uint8_t dir;        // DIR_N/E/S/W
    uint8_t cells;      // number of cells to drive straight
} PathSegment;

// ═══════════════════════════════════════════════════════════════════════════════
// State
// ═══════════════════════════════════════════════════════════════════════════════
static SpeedrunState s_state = SPEEDRUN_IDLE;
static uint8_t   s_speed_level = 0;

// Path (compressed into segments)
static PathSegment s_segs[SPEEDRUN_MAX_PATH / 2 + 1];
static uint16_t s_num_segs = 0;
static uint16_t s_seg_idx  = 0;

// Raw path (cell-by-cell directions for total length tracking)
static uint16_t s_path_len = 0;       // total cells in path
static uint16_t s_progress = 0;       // cells completed

// Sub-motion
static MotionProfile s_profile;
static PidController s_vel_pid;
static PidController s_heading_pid;
static float   s_actual_heading = 0;
static float   s_target_heading = 0;
static float   s_actual_dist = 0;
static int16_t s_steer_trim = 0;

enum SpeedSub { SSUB_IDLE = 0, SSUB_DRIVE, SSUB_TURN };
static SpeedSub s_sub = SSUB_IDLE;
static uint32_t s_tick_count = 0;
static uint8_t  s_heading = 0;        // current heading during run

static bool s_pid_inited = false;

// ═══════════════════════════════════════════════════════════════════════════════
// Speed level → velocity
// ═══════════════════════════════════════════════════════════════════════════════
static float speed_vel(void) {
    switch (s_speed_level) {
        case 1:  return (float)SPEEDRUN_VEL_1_MMPS;
        case 2:  return (float)SPEEDRUN_VEL_2_MMPS;
        case 3:  return (float)SPEEDRUN_VEL_3_MMPS;
        default: return (float)SPEEDRUN_VEL_0_MMPS;
    }
}

static float speed_accel(void) { return (float)SPEEDRUN_ACCEL_MMPS2; }
static float speed_decel(void) { return (float)SPEEDRUN_DECEL_MMPS2; }
static float speed_turn_dps(void) { return (float)SPEEDRUN_TURN_DPS; }

// ═══════════════════════════════════════════════════════════════════════════════
// Init PID
// ═══════════════════════════════════════════════════════════════════════════════
static void ensure_init(void) {
    if (s_pid_inited) return;
    s_pid_inited = true;

    pid_init(&s_vel_pid,
             PD_LINEAR_KP_X100 / 100.0f, 0,
             PD_LINEAR_KD_X100 / 100.0f,
             PD_LINEAR_MAX_MV);

    pid_init(&s_heading_pid,
             PD_STEER_KP_X100 / 100.0f,
             PD_STEER_KI_X100 / 100.0f,
             PD_STEER_KD_X100 / 100.0f,
             PD_STEER_MAX_MV);

    s_steer_trim = FF_STEERING_TRIM_MV;
}

// ═══════════════════════════════════════════════════════════════════════════════
// Plan path: flood → trace → compress into segments
// ═══════════════════════════════════════════════════════════════════════════════
static bool plan_path(void) {
    const Maze* maze = lab5_get_maze();
    if (!maze) return false;

    // Make a local copy of maze for planning
    // (We need flood values, which may have changed since search)
    Maze local;
    memcpy(&local, maze, sizeof(Maze));

    // Flood fill from goal
    maze_flood(&local);

    // Check if start is reachable
    if (local.flood[START_Y][START_X] >= FLOOD_MAX) return false;

    // Trace shortest path from start to goal
    uint8_t path_dirs[SPEEDRUN_MAX_PATH];
    uint16_t len = 0;
    uint8_t cx = START_X, cy = START_Y;

    while (len < SPEEDRUN_MAX_PATH) {
        // At goal?
        if (maze_is_goal(&local, cx, cy)) break;

        // Find best neighbor
        int8_t best = maze_best_dir(&local, cx, cy);
        if (best < 0) return false;  // stuck

        path_dirs[len++] = (uint8_t)best;

        // Move to neighbor
        uint8_t nx, ny;
        if (!maze_step(cx, cy, (uint8_t)best, &nx, &ny)) return false;
        cx = nx; cy = ny;
    }

    if (len == 0) return false;

    // Compress into segments
    s_num_segs = 0;
    uint8_t cur_dir = path_dirs[0];
    uint8_t cur_cnt = 1;

    for (uint16_t i = 1; i < len; i++) {
        if (path_dirs[i] == cur_dir) {
            cur_cnt++;
        } else {
            s_segs[s_num_segs].dir = cur_dir;
            s_segs[s_num_segs].cells = cur_cnt;
            s_num_segs++;
            cur_dir = path_dirs[i];
            cur_cnt = 1;
        }
    }
    // Last segment
    s_segs[s_num_segs].dir = cur_dir;
    s_segs[s_num_segs].cells = cur_cnt;
    s_num_segs++;

    s_path_len = len;
    return true;
}

// ═══════════════════════════════════════════════════════════════════════════════
// Start driving N cells straight
// ═══════════════════════════════════════════════════════════════════════════════
static void start_drive(uint8_t cells) {
    ensure_init();
    pid_reset(&s_vel_pid);
    pid_reset(&s_heading_pid);

    pid_set_gains(&s_heading_pid,
                  PD_STEER_KP_X100 / 100.0f,
                  PD_STEER_KI_X100 / 100.0f,
                  PD_STEER_KD_X100 / 100.0f,
                  PD_STEER_MAX_MV);

    s_actual_heading = 0;
    s_target_heading = 0;
    s_actual_dist = 0;
    s_tick_count = 0;

    float dist = CELL_SIZE_MM * cells;
    profile_plan_linear(&s_profile, dist, speed_vel(), speed_accel(), speed_decel());

    s_sub = SSUB_DRIVE;
}

// ═══════════════════════════════════════════════════════════════════════════════
// Start turn
// ═══════════════════════════════════════════════════════════════════════════════
static void start_turn(uint8_t from_dir, uint8_t to_dir) {
    int8_t diff = (int8_t)to_dir - (int8_t)from_dir;
    if (diff < 0) diff += 4;
    if (diff == 0) { s_sub = SSUB_IDLE; return; }

    float angle;
    switch (diff) {
        case 1:  angle =  90.0f; break;   // CW
        case 2:  angle = 180.0f; break;   // U-turn
        case 3:  angle = -90.0f; break;   // CCW
        default: s_sub = SSUB_IDLE; return;
    }

    ensure_init();
    pid_reset(&s_vel_pid);
    pid_reset(&s_heading_pid);

    pid_set_gains(&s_heading_pid,
                  PD_ROT_KP_X100 / 100.0f, 0,
                  PD_ROT_KD_X100 / 100.0f,
                  PD_ROT_MAX_MV);

    s_actual_heading = 0;
    s_target_heading = 0;
    s_actual_dist = 0;
    s_tick_count = 0;

    profile_plan_turn(&s_profile, angle,
                      speed_turn_dps(), (float)TURN_ACCEL_DPS2, (float)TURN_ACCEL_DPS2);

    s_sub = SSUB_TURN;
}

// ═══════════════════════════════════════════════════════════════════════════════
// Sub-motion tick
// ═══════════════════════════════════════════════════════════════════════════════
static bool sub_tick(int16_t* vL, int16_t* vR,
                     int16_t velL_mmps, int16_t velR_mmps,
                     float gyroZ_dps) {
    if (s_sub == SSUB_IDLE) {
        *vL = 0; *vR = 0;
        return false;
    }

    s_tick_count++;
    if (s_tick_count > 30000) {  // 30s timeout
        s_sub = SSUB_IDLE;
        *vL = 0; *vR = 0;
        return false;
    }

    s_actual_heading += gyroZ_dps * (float)GYRO_Z_SIGN * DT;

    if (s_sub == SSUB_DRIVE) {
        float avg_vel = (float)(velL_mmps + velR_mmps) * 0.5f;
        s_actual_dist += fabsf(avg_vel) * DT;

        float target_vel = profile_tick(&s_profile, s_actual_dist);
        float target_acc = profile_get_accel(&s_profile);

        if (profile_is_done(&s_profile)) {
            s_sub = SSUB_IDLE;
            *vL = 0; *vR = 0;
            return false;
        }

        float actual_vel = (float)(velL_mmps + velR_mmps) * 0.5f;
        float vel_corr = pid_tick(&s_vel_pid, target_vel - actual_vel);
        float steer_corr = pid_tick(&s_heading_pid, s_target_heading - s_actual_heading);

        float ff_L = (float)ff_voltage_L(target_vel, target_acc);
        float ff_R = (float)ff_voltage_R(target_vel, target_acc);

        *vL = clamp_i16(ff_L + vel_corr - steer_corr + s_steer_trim, -VCMD_MAX_MV, VCMD_MAX_MV);
        *vR = clamp_i16(ff_R + vel_corr + steer_corr - s_steer_trim, -VCMD_MAX_MV, VCMD_MAX_MV);

    } else if (s_sub == SSUB_TURN) {
        s_actual_dist = fabsf(s_actual_heading);

        float target_omega = profile_tick(&s_profile, s_actual_dist);
        float target_alpha = profile_get_accel(&s_profile);
        s_target_heading += target_omega * DT;

        if (profile_is_done(&s_profile)) {
            s_sub = SSUB_IDLE;
            *vL = 0; *vR = 0;
            return false;
        }

        float heading_corr = pid_tick(&s_heading_pid,
                                      s_target_heading - s_actual_heading);

        float omega_rad = target_omega * (3.14159265f / 180.0f);
        float v_wheel = omega_rad * WHEEL_BASE_MM * 0.5f;
        float a_wheel = target_alpha * (3.14159265f / 180.0f) * WHEEL_BASE_MM * 0.5f;

        float ff_L = (float)ff_voltage_L( v_wheel,  a_wheel);
        float ff_R = (float)ff_voltage_R(-v_wheel, -a_wheel);

        *vL = clamp_i16(ff_L + heading_corr, -VCMD_MAX_MV, VCMD_MAX_MV);
        *vR = clamp_i16(ff_R - heading_corr, -VCMD_MAX_MV, VCMD_MAX_MV);
    }

    return true;
}

// ═══════════════════════════════════════════════════════════════════════════════
// Start next segment (drive or turn+drive)
// ═══════════════════════════════════════════════════════════════════════════════
static void start_next_segment(void) {
    if (s_seg_idx >= s_num_segs) {
        s_state = SPEEDRUN_DONE;
        return;
    }

    PathSegment* seg = &s_segs[s_seg_idx];

    if (seg->dir != s_heading) {
        // Need to turn first
        s_state = SPEEDRUN_TURNING;
        start_turn(s_heading, seg->dir);
        s_heading = seg->dir;
    } else {
        // Same direction — drive straight
        s_state = SPEEDRUN_RUNNING;
        start_drive(seg->cells);
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
// Public: handle test command
// ═══════════════════════════════════════════════════════════════════════════════
bool lab6_handle_test(const CommandRequest* cmd, CommandResult* result,
                      int16_t* vCmdL_mV, int16_t* vCmdR_mV) {
    uint8_t test_id = cmd->payload[0];
    memset(result, 0, sizeof(*result));
    result->opcode = cmd->opcode;
    result->req_id = cmd->req_id;
    result->status = RSP_OK;

    switch (test_id) {
    case TEST_SPEEDRUN_START: {
        s_speed_level = cmd->payload[1];
        if (s_speed_level > 3) s_speed_level = 3;

        // Plan path from Lab 5 maze
        if (!plan_path()) {
            result->status = RSP_ERR_BAD_ARG;  // no maze or no path
            s_state = SPEEDRUN_ERROR;
            *vCmdL_mV = 0; *vCmdR_mV = 0;
            return true;
        }

        // Start execution
        s_seg_idx = 0;
        s_progress = 0;
        s_heading = lab5_get_pose()->heading;
        ensure_init();
        start_next_segment();
        *vCmdL_mV = 0; *vCmdR_mV = 0;
        return true;
    }
    case TEST_SPEEDRUN_STOP: {
        lab6_stop();
        *vCmdL_mV = 0; *vCmdR_mV = 0;
        return true;
    }
    case TEST_SPEEDRUN_QUERY: {
        // RSP: [0] state [1] speed_level [2-3] path_len [4-5] progress
        //      [6-7] num_segments [8-9] current_seg
        uint8_t* rp = result->rsp_payload;
        rp[0] = (uint8_t)s_state;
        rp[1] = s_speed_level;
        rp[2] = (uint8_t)(s_path_len & 0xFF);
        rp[3] = (uint8_t)(s_path_len >> 8);
        rp[4] = (uint8_t)(s_progress & 0xFF);
        rp[5] = (uint8_t)(s_progress >> 8);
        rp[6] = (uint8_t)(s_num_segs & 0xFF);
        rp[7] = (uint8_t)(s_num_segs >> 8);
        rp[8] = (uint8_t)(s_seg_idx & 0xFF);
        rp[9] = (uint8_t)(s_seg_idx >> 8);
        result->rsp_payload_len = 10;
        *vCmdL_mV = 0; *vCmdR_mV = 0;
        return true;
    }
    default:
        return false;
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
// Public: tick @ 1kHz
// ═══════════════════════════════════════════════════════════════════════════════
bool lab6_tick(int16_t* vCmdL_mV, int16_t* vCmdR_mV,
               int16_t velL_mmps, int16_t velR_mmps,
               float gyroZ_dps) {

    if (s_state == SPEEDRUN_IDLE || s_state == SPEEDRUN_DONE ||
        s_state == SPEEDRUN_ERROR) {
        *vCmdL_mV = 0; *vCmdR_mV = 0;
        return false;
    }

    // Turning
    if (s_state == SPEEDRUN_TURNING) {
        bool running = sub_tick(vCmdL_mV, vCmdR_mV,
                                velL_mmps, velR_mmps, gyroZ_dps);
        if (!running) {
            // Turn done — now drive this segment
            s_state = SPEEDRUN_RUNNING;
            start_drive(s_segs[s_seg_idx].cells);
        }
        return true;
    }

    // Driving
    if (s_state == SPEEDRUN_RUNNING) {
        bool running = sub_tick(vCmdL_mV, vCmdR_mV,
                                velL_mmps, velR_mmps, gyroZ_dps);
        if (!running) {
            // Segment done — advance
            s_progress += s_segs[s_seg_idx].cells;
            s_seg_idx++;

            if (s_seg_idx >= s_num_segs) {
                s_state = SPEEDRUN_DONE;
                *vCmdL_mV = 0; *vCmdR_mV = 0;
                return false;
            }
            start_next_segment();
        }
        return true;
    }

    *vCmdL_mV = 0; *vCmdR_mV = 0;
    return true;
}

// ═══════════════════════════════════════════════════════════════════════════════
// Public: stop / query
// ═══════════════════════════════════════════════════════════════════════════════
void lab6_stop(void) {
    s_state = SPEEDRUN_IDLE;
    s_sub = SSUB_IDLE;
    profile_reset(&s_profile);
}

bool lab6_is_running(void) {
    return s_state == SPEEDRUN_RUNNING || s_state == SPEEDRUN_TURNING;
}

SpeedrunState lab6_get_state(void)    { return s_state; }
uint16_t      lab6_get_path_len(void) { return s_path_len; }
uint16_t      lab6_get_progress(void) { return s_progress; }
