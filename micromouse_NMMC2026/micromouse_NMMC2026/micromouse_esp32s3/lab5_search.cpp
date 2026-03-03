// ═══════════════════════════════════════════════════════════════════════════════
//  lab5_search.cpp — Lab 5: Maze Search Implementation
//
//  Two modes:
//    STOP-AND-GO:  stop every cell → read → decide → move
//    CONTINUOUS:    multi-cell straights, stop only before turns
//
//  BUG FIXES:
//    - Heading init: use DIR_N directly (not START_HEADING_DEG/90)
//    - STEP resets on ERROR/DONE states too
//    - Boundary walls never cleared by sensor readings
//    - Debug: stores last sensor readings for GUI
// ═══════════════════════════════════════════════════════════════════════════════
#include "lab5_search.h"
#include "maze.h"
#include "pid.h"
#include "motion_profile.h"
#include "feedforward.h"
#include "config.h"
#include "hal_wallsensor.h"
#include <string.h>
#include <math.h>

extern int16_t g_v_target_mmps;  // set by active lab for telemetry

// ── Helpers ──────────────────────────────────────────────────────────────────
static inline int16_t clamp_i16(float v, int16_t lo, int16_t hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return (int16_t)v;
}

#define DT          0.001f

// ═══════════════════════════════════════════════════════════════════════════════
// State
// ═══════════════════════════════════════════════════════════════════════════════
static Maze         s_maze;
static RobotPose    s_pose;
static SearchState  s_state = SEARCH_IDLE;
static SearchMode   s_mode  = SEARCH_MODE_STOP_GO;
static bool         s_maze_inited = false;   // ★ tracks if maze/pose has been initialized

// ★ Configurable start/goal (GUI can override via SET_CONFIG before search)
static uint8_t s_start_x = START_X;
static uint8_t s_start_y = START_Y;
static uint8_t s_start_heading_deg = START_HEADING_DEG;
static uint8_t s_goal_x_min = 7, s_goal_y_min = 7;
static uint8_t s_goal_x_max = 8, s_goal_y_max = 8;

// Sub-motion control
static MotionProfile s_profile;
static PidController s_vel_pid;
static PidController s_heading_pid;
static float   s_actual_heading  = 0;
static float   s_target_heading  = 0;
static float   s_actual_dist     = 0;
static int16_t s_steer_trim      = 0;

enum SubMotion { SUB_IDLE = 0, SUB_TURN, SUB_DRIVE, SUB_WALL_FOLLOW };
static SubMotion s_sub = SUB_IDLE;

// Search control
static bool  s_continuous   = false;
static bool  s_return_mode  = false;

// Wall follow control
static int8_t  s_wall_follow_side = 0;  // -1=left, +1=right, 0=off
static PidController s_wall_pid;

// Timeout
static uint32_t s_tick_count = 0;
#define SEARCH_MOVE_TIMEOUT  5000
#define WARMUP_TICKS         50     // 50ms sensor warmup before first decision

// Continuous mode state
static float   s_cont_vel         = 0;
static float   s_cont_dist_cell   = 0;
static bool    s_cont_stopping    = false;
static float   s_cont_heading_acc = 0;
static uint32_t s_cont_timeout    = 0;

// ═══════════════════════════════════════════════════════════════════════════════
// Debug info (readable by GUI via telemetry)
// ═══════════════════════════════════════════════════════════════════════════════
static int16_t  s_dbg_wall_mm[4]  = {0,0,0,0};  // L, FL, FR, R in mm
static bool     s_dbg_wall_det[4] = {0,0,0,0};   // detected as wall?
static uint8_t  s_dbg_error_code  = 0;            // 0=ok, 1=stuck, 2=timeout

// Accessors for telemetry
int16_t  lab5_dbg_wall_mm(uint8_t i)  { return (i<4) ? s_dbg_wall_mm[i]  : -1; }
bool     lab5_dbg_wall_det(uint8_t i)  { return (i<4) ? s_dbg_wall_det[i] : false; }
uint8_t  lab5_dbg_error_code(void)     { return s_dbg_error_code; }

// ═══════════════════════════════════════════════════════════════════════════════
// Front Safety System state
// ═══════════════════════════════════════════════════════════════════════════════
static uint8_t  s_front_estop_cnt = 0;   // consecutive emergency readings
static bool     s_front_braking   = false; // progressive brake is active (for debug)

// ═══════════════════════════════════════════════════════════════════════════════
// Wall Detector — hysteresis + debounce (runs at 1kHz in tick)
//
//  ★ CODE REVIEW POINTER:
//    เงื่อนไข "เจอกำแพงหรือไม่" อยู่ใน wall_detect_sample() ด้านล่าง
//    - ใช้ filtered distance (mm) จาก hal_wall_get_mm()
//    - distance < ON_threshold → want_wall = true
//    - distance > OFF_threshold → want_wall = false
//    - ON < distance < OFF → คง state เดิม (hysteresis dead zone)
//    - ต้องอ่านค่าตรงกัน WALL_DEBOUNCE_COUNT ครั้งติดต่อกันจึงเปลี่ยน state
//
//  ★ สำหรับ front sensors (คู่ FL+FR):
//    เงื่อนไขรวมอยู่ใน wall_detect_get_front() ด้านล่าง
//    เลือก logic ได้ผ่าน WALL_FRONT_DETECT_MODE ใน config.h
//
//  ★ FILE/FUNCTION/LINE REFERENCE:
//    - config.h:116-170  → threshold ค่า + mode
//    - lab5_search.cpp:wall_detect_sample()  → single sensor hysteresis+debounce
//    - lab5_search.cpp:wall_detect_get_front() → front pair combine logic
//    - lab5_search.cpp:read_and_update_walls() → maps detections → maze walls
// ═══════════════════════════════════════════════════════════════════════════════

// Per-sensor detector state
typedef struct {
    bool     state;         // current debounced wall state
    bool     raw_want;      // what hysteresis says this tick
    uint8_t  counter;       // debounce counter (counts toward change)
    int16_t  last_mm;       // last raw reading (mm)
    int16_t  on_mm;         // threshold to detect wall (distance < on)
    int16_t  off_mm;        // threshold to release wall (distance > off)
} WallDetector;

#define WD_IDX_L   0
#define WD_IDX_FL  1
#define WD_IDX_FR  2
#define WD_IDX_R   3

static WallDetector s_wd[4];

static void wall_detect_init(void) {
    // Left
    s_wd[WD_IDX_L]  = (WallDetector){false, false, 0, 999, WALL_HYST_SL_ON_MM, WALL_HYST_SL_OFF_MM};
    // Front-Left
    s_wd[WD_IDX_FL] = (WallDetector){false, false, 0, 999, WALL_HYST_FL_ON_MM, WALL_HYST_FL_OFF_MM};
    // Front-Right
    s_wd[WD_IDX_FR] = (WallDetector){false, false, 0, 999, WALL_HYST_FR_ON_MM, WALL_HYST_FR_OFF_MM};
    // Right
    s_wd[WD_IDX_R]  = (WallDetector){false, false, 0, 999, WALL_HYST_SR_ON_MM, WALL_HYST_SR_OFF_MM};
}

// ★ CORE DECISION: single sensor → wall or not?
static void wall_detect_sample(WallDetector* wd, int16_t mm) {
    wd->last_mm = mm;

    // Hysteresis logic
    bool want;
    if (mm < wd->on_mm) {
        want = true;           // clearly a wall
    } else if (mm > wd->off_mm) {
        want = false;          // clearly no wall
    } else {
        want = wd->state;      // in dead zone → keep current state
    }
    wd->raw_want = want;

    // Debounce: only change state after N consecutive agreements
    if (want != wd->state) {
        wd->counter++;
        if (wd->counter >= WALL_DEBOUNCE_COUNT) {
            wd->state = want;
            wd->counter = 0;
        }
    } else {
        wd->counter = 0;       // reset if reading agrees with state
    }
}

// Sample all 4 sensors (call at 1kHz from tick)
static void wall_detect_tick_all(void) {
    wall_detect_sample(&s_wd[WD_IDX_L],  hal_wall_get_mm(SNS_POS_LEFT));
    wall_detect_sample(&s_wd[WD_IDX_FL], hal_wall_get_mm(SNS_POS_FRONT_LEFT));
    wall_detect_sample(&s_wd[WD_IDX_FR], hal_wall_get_mm(SNS_POS_FRONT_RIGHT));
    wall_detect_sample(&s_wd[WD_IDX_R],  hal_wall_get_mm(SNS_POS_RIGHT));

    // Update debug arrays
    s_dbg_wall_mm[0] = s_wd[WD_IDX_L].last_mm;
    s_dbg_wall_mm[1] = s_wd[WD_IDX_FL].last_mm;
    s_dbg_wall_mm[2] = s_wd[WD_IDX_FR].last_mm;
    s_dbg_wall_mm[3] = s_wd[WD_IDX_R].last_mm;
    s_dbg_wall_det[0] = s_wd[WD_IDX_L].state;
    s_dbg_wall_det[1] = s_wd[WD_IDX_FL].state;
    s_dbg_wall_det[2] = s_wd[WD_IDX_FR].state;
    s_dbg_wall_det[3] = s_wd[WD_IDX_R].state;
}

// ★ FRONT PAIR COMBINE LOGIC (selectable via WALL_FRONT_DETECT_MODE)
static bool wall_detect_get_front(void) {
    bool fl = s_wd[WD_IDX_FL].state;
    bool fr = s_wd[WD_IDX_FR].state;

    #if WALL_FRONT_DETECT_MODE == 1
        return fl && fr;                          // AND: both agree
    #elif WALL_FRONT_DETECT_MODE == 2
        // AVG: use average of raw mm vs average of ON thresholds
        return ((s_wd[WD_IDX_FL].last_mm + s_wd[WD_IDX_FR].last_mm) / 2) <
               ((WALL_HYST_FL_ON_MM + WALL_HYST_FR_ON_MM) / 2);
    #elif WALL_FRONT_DETECT_MODE == 3
        // MIN: use the sensor closer to wall (smaller reading = more likely wall)
        { int16_t mn = (s_wd[WD_IDX_FL].last_mm < s_wd[WD_IDX_FR].last_mm)
                       ? s_wd[WD_IDX_FL].last_mm : s_wd[WD_IDX_FR].last_mm;
          int16_t th = (WALL_HYST_FL_ON_MM < WALL_HYST_FR_ON_MM)
                       ? WALL_HYST_FL_ON_MM : WALL_HYST_FR_ON_MM;
          return mn < th; }
    #elif WALL_FRONT_DETECT_MODE == 4
        return fl;                                // FL only
    #elif WALL_FRONT_DETECT_MODE == 5
        return fr;                                // FR only
    #else
        return fl || fr;                          // Mode 0: OR (safest ★default)
    #endif
}

// ═══════════════════════════════════════════════════════════════════════════════
// Convert START_HEADING_DEG to DIR_* safely
// ═══════════════════════════════════════════════════════════════════════════════
static uint8_t heading_deg_to_dir(int deg) {
    // Normalize to 0-359
    deg = ((deg % 360) + 360) % 360;
    // Math convention (from config.h): 0°=East, 90°=North, 180°=West, 270°=South
    if (deg >= 45  && deg < 135) return DIR_N;   // 90° ± 45 → North
    if (deg >= 135 && deg < 225) return DIR_W;   // 180° ± 45 → West
    if (deg >= 225 && deg < 315) return DIR_S;   // 270° ± 45 → South
    return DIR_E;                                 // 0°/360° ± 45 → East
}

// ═══════════════════════════════════════════════════════════════════════════════
// Wall Centering — use side wall distances to keep robot centered
//
// Returns a steering correction (deg) to add to heading target:
//   positive = turn left (robot drifting right, push it left)
//   negative = turn right
//
// Three modes:
//   Both walls  → error = (left - right) * BOTH_GAIN  (balanced centering)
//   Left only   → error = (left - target) * ONE_GAIN
//   Right only  → error = -(right - target) * ONE_GAIN
//   No walls    → error = 0 (gyro only)
// ═══════════════════════════════════════════════════════════════════════════════
static float wall_center_error(void) {
#if WALL_CENTER_ENABLE
    bool has_left  = s_wd[WD_IDX_L].state;
    bool has_right = s_wd[WD_IDX_R].state;
    float left_mm  = (float)s_wd[WD_IDX_L].last_mm;
    float right_mm = (float)s_wd[WD_IDX_R].last_mm;
    float target   = (float)WALL_CENTER_TARGET_MM;

    if (has_left && has_right) {
        // Both walls: center between them
        // Positive error = left closer than right = drifting left = need to go right
        return (left_mm - right_mm) * WALL_CENTER_BOTH_GAIN;
    } else if (has_left) {
        // Left wall only: maintain target distance from left
        // If left_mm < target → too close to left → positive → steer right
        return (target - left_mm) * WALL_CENTER_ONE_GAIN;
    } else if (has_right) {
        // Right wall only: maintain target distance from right
        // If right_mm < target → too close to right → negative → steer left
        return -(target - right_mm) * WALL_CENTER_ONE_GAIN;
    }
#endif
    return 0.0f;  // no walls → gyro only
}

// ═══════════════════════════════════════════════════════════════════════════════
// Init PID controllers
// ═══════════════════════════════════════════════════════════════════════════════
static bool s_pid_inited = false;

static void ensure_init(void) {
    if (s_pid_inited) return;
    s_pid_inited = true;

    pid_init(&s_vel_pid,
             PD_LINEAR_KP_X100 / 100.0f,
             0,
             PD_LINEAR_KD_X100 / 100.0f,
             PD_LINEAR_MAX_MV);

    pid_init(&s_heading_pid,
             PD_STEER_KP_X100 / 100.0f,
             PD_STEER_KI_X100 / 100.0f,
             PD_STEER_KD_X100 / 100.0f,
             PD_STEER_MAX_MV);

    pid_init(&s_wall_pid,
             PD_WALL_KP_X100 / 100.0f,
             0,
             PD_WALL_KD_X100 / 100.0f,
             PD_WALL_MAX_MV);

    s_steer_trim = FF_STEERING_TRIM_MV;
}

// ═══════════════════════════════════════════════════════════════════════════════
// Reset everything for a fresh search
// ═══════════════════════════════════════════════════════════════════════════════
static void full_reset(void) {
    maze_init(&s_maze);
    // ★ Apply GUI-configured goal (overrides maze_init defaults)
    s_maze.goal_x_min = s_goal_x_min;
    s_maze.goal_y_min = s_goal_y_min;
    s_maze.goal_x_max = s_goal_x_max;
    s_maze.goal_y_max = s_goal_y_max;
    // ★ Mark start cell as visited immediately (matches mazerunner-core).
    maze_set_visited(&s_maze, s_start_x, s_start_y);
    s_pose.x = s_start_x;
    s_pose.y = s_start_y;
    s_pose.heading = heading_deg_to_dir(s_start_heading_deg);
    s_pose.steps = 0;
    s_dbg_error_code = 0;
    s_front_estop_cnt = 0;
    s_front_braking = false;
    wall_detect_init();
    ensure_init();
    s_maze_inited = true;
}

// ═══════════════════════════════════════════════════════════════════════════════
// Wall reading — map sensors to maze walls based on heading
//
// ★ FIX: Never clear boundary walls (row 0/15, col 0/15 outer edges)
// ═══════════════════════════════════════════════════════════════════════════════
static bool is_boundary_wall(uint8_t x, uint8_t y, uint8_t dir) {
    if (dir == DIR_N && y == MAZE_SIZE - 1) return true;
    if (dir == DIR_S && y == 0)             return true;
    if (dir == DIR_E && x == MAZE_SIZE - 1) return true;
    if (dir == DIR_W && x == 0)             return true;
    return false;
}

// ── Safe wall update (prevents reciprocal conflicts) ─────────────────────────
// Matches mazerunner-core's update_wall_state() semantics:
//   - Setting a wall is always safe (both sides get the wall)
//   - Clearing a wall checks that the neighbor hasn't already CONFIRMED the wall
//     from their side (i.e., neighbor visited + wall bit set = locked)
//   - Boundary walls are NEVER cleared
static void safe_update_wall(Maze* m, uint8_t x, uint8_t y, uint8_t dir, bool has_wall) {
    if (is_boundary_wall(x, y, dir)) {
        // Boundary walls are always there — only allow setting, never clearing
        if (has_wall) maze_set_wall(m, x, y, dir);
        return;
    }
    if (has_wall) {
        maze_set_wall(m, x, y, dir);
    } else {
        // Before clearing: check if neighbor already confirmed this wall
        uint8_t nx, ny;
        if (maze_step(x, y, dir, &nx, &ny)) {
            uint8_t opp = dir_opposite(dir);
            if (maze_is_visited(m, nx, ny) && maze_has_wall(m, nx, ny, opp)) {
                // Neighbor visited and confirmed wall exists — keep it!
                // (sensor disagreement: trust the earlier observation)
                return;
            }
        }
        maze_clear_wall(m, x, y, dir);
    }
}

static void read_and_update_walls(void) {
    uint8_t x = s_pose.x;
    uint8_t y = s_pose.y;
    uint8_t h = s_pose.heading;

    // ★ KEY FIX (from mazerunner-core update_wall_state):
    //   Only update walls for cells that haven't been visited yet.
    //   Once a cell is visited, its walls are LOCKED — never changed again.
    //   This prevents wall toggling from sensor noise and matches the
    //   reference: "once seen, a wall should not be changed again."
    if (maze_is_visited(&s_maze, x, y)) return;

    // ★ Wall states come from the debounced WallDetector (sampled at 1kHz)
    //   No need to read sensors here — wall_detect_tick_all() does it continuously
    bool wall_front = wall_detect_get_front();
    bool wall_left  = s_wd[WD_IDX_L].state;
    bool wall_right = s_wd[WD_IDX_R].state;

    // heading=N(0): front=N, left=W, right=E
    uint8_t dir_front = h;
    uint8_t dir_left  = (h + 3) & 3;   // CCW 90°
    uint8_t dir_right = (h + 1) & 3;   // CW 90°

    safe_update_wall(&s_maze, x, y, dir_front, wall_front);
    safe_update_wall(&s_maze, x, y, dir_left,  wall_left);
    safe_update_wall(&s_maze, x, y, dir_right, wall_right);

    maze_set_visited(&s_maze, x, y);
}

// ═══════════════════════════════════════════════════════════════════════════════
// Check if we can continue straight (continuous mode)
// ═══════════════════════════════════════════════════════════════════════════════
static bool can_continue_straight(void) {
    uint8_t x = s_pose.x, y = s_pose.y, h = s_pose.heading;

    if (maze_has_wall(&s_maze, x, y, h)) return false;

    if (!s_return_mode && maze_is_goal(&s_maze, x, y)) return false;
    if (s_return_mode && x == s_start_x && y == s_start_y) return false;

    if (s_return_mode) {
        maze_flood_target(&s_maze, s_start_x, s_start_y);
    } else {
        maze_flood(&s_maze);
    }
    int8_t best = maze_best_dir(&s_maze, x, y);
    return (best == (int8_t)h);
}

// ═══════════════════════════════════════════════════════════════════════════════
// Sub-motion: start turn
// ═══════════════════════════════════════════════════════════════════════════════
static void start_turn(int8_t turn_type) {
    if (turn_type == 0) {
        s_sub = SUB_IDLE;
        return;
    }

    float angle_deg;
    switch (turn_type) {
        case 1:  angle_deg =  90.0f;  break;  // CW
        case 2:  angle_deg = 180.0f;  break;  // U-turn
        case 3:  angle_deg = -90.0f;  break;  // CCW
        default: s_sub = SUB_IDLE; return;
    }

    ensure_init();
    pid_reset(&s_vel_pid);
    pid_reset(&s_heading_pid);

    pid_set_gains(&s_heading_pid,
                  PD_ROT_KP_X100 / 100.0f,
                  0,
                  PD_ROT_KD_X100 / 100.0f,
                  PD_ROT_MAX_MV);

    s_actual_heading = 0;
    s_target_heading = 0;
    s_actual_dist = 0;
    s_tick_count = 0;

    profile_plan_turn(&s_profile, angle_deg,
                      (float)TURN_VEL_DPS, (float)TURN_ACCEL_DPS2, (float)TURN_ACCEL_DPS2);

    s_sub = SUB_TURN;
}

// ═══════════════════════════════════════════════════════════════════════════════
// Sub-motion: start single-cell drive (stop-and-go)
// ═══════════════════════════════════════════════════════════════════════════════
static void start_drive_single(void) {
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

    profile_plan_linear(&s_profile, CELL_SIZE_MM,
                        (float)SEARCH_VEL_MMPS,
                        (float)SEARCH_ACCEL_MMPS2,
                        (float)SEARCH_DECEL_MMPS2);

    s_sub = SUB_DRIVE;
}

// ═══════════════════════════════════════════════════════════════════════════════
// Front Safety: Progressive Brake Limit
//
//  Returns max allowed velocity (mm/s) based on front sensor distance.
//  Uses 3-stage linear interpolation tuned for SHARP IR (10-150mm) with
//  sensor mounted 64mm ahead of robot center.
//
//  Profile:  ≥TH1(80mm) → full | TH1→TH2 → 50% | TH2→TH3 → 25% | <TH3 → creep
// ═══════════════════════════════════════════════════════════════════════════════
static inline int16_t min_i16(int16_t a, int16_t b) { return (a < b) ? a : b; }

#if FRONT_BRAKE_ENABLE
static float front_brake_limit(int16_t front_mm) {
    const float v_full = (float)SEARCH_VEL_MMPS;
    const float v_creep = (float)FRONT_BRAKE_CREEP_MMPS;

    if (front_mm >= FRONT_BRAKE_TH1_MM) return v_full;

    if (front_mm >= FRONT_BRAKE_TH2_MM) {
        // TH1 → TH2: full → 50%
        float t = (float)(FRONT_BRAKE_TH1_MM - front_mm)
                / (float)(FRONT_BRAKE_TH1_MM - FRONT_BRAKE_TH2_MM);
        return v_full * (1.0f - 0.5f * t);   // 100% → 50%
    }
    if (front_mm >= FRONT_BRAKE_TH3_MM) {
        // TH2 → TH3: 50% → creep
        float t = (float)(FRONT_BRAKE_TH2_MM - front_mm)
                / (float)(FRONT_BRAKE_TH2_MM - FRONT_BRAKE_TH3_MM);
        float v_half = v_full * 0.5f;
        return v_half + (v_creep - v_half) * t;  // 50% → creep
    }
    return v_creep;
}
#endif

// ═══════════════════════════════════════════════════════════════════════════════
// Sub-motion tick (TURN and single-cell DRIVE)
// ═══════════════════════════════════════════════════════════════════════════════
static bool sub_motion_tick(int16_t* vL, int16_t* vR,
                            int16_t velL_mmps, int16_t velR_mmps,
                            float gyroZ_dps) {
    if (s_sub == SUB_IDLE) {
        *vL = 0; *vR = 0;
        return false;
    }

    s_tick_count++;
    if (s_tick_count > SEARCH_MOVE_TIMEOUT) {
        s_sub = SUB_IDLE;
        s_dbg_error_code = 2;  // timeout
        *vL = 0; *vR = 0;
        return false;
    }

    // Integrate gyro
    s_actual_heading += gyroZ_dps * (float)GYRO_Z_SIGN * DT;

    if (s_sub == SUB_DRIVE) {
        float avg_vel = (float)(velL_mmps + velR_mmps) * 0.5f;
        s_actual_dist += fabsf(avg_vel) * DT;

        // ═════════════════════════════════════════════════════════════
        // ★ LAYER 1: EMERGENCY STOP — front sensor < 15mm
        //   Uses raw mm from WallDetector (updated every tick via
        //   wall_detect_tick_all). Sensor hardware updates at ~30Hz,
        //   but checking every tick catches it ASAP.
        // ═════════════════════════════════════════════════════════════
#if FRONT_ESTOP_ENABLE
        {
            int16_t front_mm = min_i16(s_wd[WD_IDX_FL].last_mm,
                                       s_wd[WD_IDX_FR].last_mm);
            if (front_mm < FRONT_ESTOP_MM && front_mm > 0) {
                s_front_estop_cnt++;
                if (s_front_estop_cnt >= FRONT_ESTOP_COUNT) {
                    // HARD STOP — collision averted
                    profile_reset(&s_profile);
                    s_sub = SUB_IDLE;
                    s_front_estop_cnt = 0;
                    s_front_braking = false;
                    s_dbg_error_code = 3;  // front emergency stop
                    s_state = SEARCH_ERROR;
                    *vL = 0; *vR = 0;
                    return false;
                }
            } else {
                s_front_estop_cnt = 0;
            }
        }
#endif

        // ═════════════════════════════════════════════════════════════
        // ★ LAYER 2: PROGRESSIVE BRAKE — clamp target_vel by front distance
        //   Smooth deceleration: 80mm→50% → 50mm→creep → 35mm→creep
        //   Does NOT modify profile — only clamps the output velocity.
        //   Profile may "finish" before reaching cell center if wall
        //   is detected; emergency stop catches that case.
        // ═════════════════════════════════════════════════════════════
        float target_vel = profile_tick(&s_profile, s_actual_dist);
        float target_acc = profile_get_accel(&s_profile);

#if FRONT_BRAKE_ENABLE
        {
            int16_t front_mm = min_i16(s_wd[WD_IDX_FL].last_mm,
                                       s_wd[WD_IDX_FR].last_mm);
            float limit = front_brake_limit(front_mm);
            if (target_vel > limit) {
                target_vel = limit;
                s_front_braking = true;
                // Also reduce acceleration when braking to avoid jerk
                if (target_acc > 0) target_acc = 0;
            } else {
                s_front_braking = false;
            }
        }
#endif

        g_v_target_mmps = (int16_t)target_vel;  // expose for telemetry

        if (profile_is_done(&s_profile)) {
            s_sub = SUB_IDLE;
            s_front_estop_cnt = 0;
            s_front_braking = false;
            *vL = 0; *vR = 0;
            return false;
        }

        float actual_vel = (float)(velL_mmps + velR_mmps) * 0.5f;
        float vel_corr = pid_tick(&s_vel_pid, target_vel - actual_vel);

        // ★ Wall centering: compute steering target from wall sensors
        float wall_err = wall_center_error();
        float wall_corr = pid_tick(&s_wall_pid, wall_err);

        // Heading PID: gyro-based + wall centering offset
        float heading_err = s_target_heading - s_actual_heading;
        float steer_corr = pid_tick(&s_heading_pid, heading_err) + wall_corr;

        float ff_L = (float)ff_voltage_L(target_vel, target_acc);
        float ff_R = (float)ff_voltage_R(target_vel, target_acc);

        *vL = clamp_i16(ff_L + vel_corr - steer_corr + s_steer_trim, -VCMD_MAX_MV, VCMD_MAX_MV);
        *vR = clamp_i16(ff_R + vel_corr + steer_corr - s_steer_trim, -VCMD_MAX_MV, VCMD_MAX_MV);

    } else if (s_sub == SUB_TURN) {
        s_actual_dist = fabsf(s_actual_heading);

        float target_omega = profile_tick(&s_profile, s_actual_dist);
        float target_alpha = profile_get_accel(&s_profile);
        g_v_target_mmps = 0;  // turns: angular target
        s_target_heading += target_omega * DT;

        if (profile_is_done(&s_profile)) {
            s_sub = SUB_IDLE;
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
// Wall Follow Tick (test mode — drive with wall centering, stop at front wall)
//
// Left-hand rule:  priority = left open > front open > right open > U-turn
// Right-hand rule: priority = right open > front open > left open > U-turn
// ═══════════════════════════════════════════════════════════════════════════════
enum WallFollowPhase { WF_DRIVE, WF_TURNING, WF_STOPPED };
static WallFollowPhase s_wf_phase = WF_DRIVE;

static bool wall_follow_tick(int16_t* vL, int16_t* vR,
                              int16_t velL_mmps, int16_t velR_mmps,
                              float gyroZ_dps) {
    if (s_sub != SUB_WALL_FOLLOW) return false;

    s_tick_count++;
    if (s_tick_count > 60000) {  // 60s timeout
        s_sub = SUB_IDLE;
        s_dbg_error_code = 2;
        *vL = 0; *vR = 0;
        return false;
    }

    s_actual_heading += gyroZ_dps * (float)GYRO_Z_SIGN * DT;

    if (s_wf_phase == WF_TURNING) {
        // Execute planned turn using profile
        s_actual_dist = fabsf(s_actual_heading - s_target_heading);
        // Use a simple approach: track heading difference
        float target_omega = profile_tick(&s_profile, s_actual_dist);
        s_target_heading += target_omega * DT;

        if (profile_is_done(&s_profile)) {
            // Turn done — start driving again
            s_wf_phase = WF_DRIVE;
            pid_reset(&s_vel_pid);
            pid_reset(&s_heading_pid);
            pid_reset(&s_wall_pid);
            s_actual_heading = 0;
            s_target_heading = 0;
            *vL = 0; *vR = 0;
            return true;
        }

        float heading_corr = pid_tick(&s_heading_pid,
                                       s_target_heading - s_actual_heading);
        float omega_rad = target_omega * (3.14159265f / 180.0f);
        float v_wheel = omega_rad * WHEEL_BASE_MM * 0.5f;
        float a_wheel = profile_get_accel(&s_profile) * (3.14159265f / 180.0f) * WHEEL_BASE_MM * 0.5f;
        float ff_L = (float)ff_voltage_L( v_wheel,  a_wheel);
        float ff_R = (float)ff_voltage_R(-v_wheel, -a_wheel);
        *vL = clamp_i16(ff_L + heading_corr, -VCMD_MAX_MV, VCMD_MAX_MV);
        *vR = clamp_i16(ff_R - heading_corr, -VCMD_MAX_MV, VCMD_MAX_MV);
        return true;
    }

    // WF_DRIVE phase: drive straight with wall centering
    bool front = wall_detect_get_front();
    bool left  = s_wd[WD_IDX_L].state;
    bool right = s_wd[WD_IDX_R].state;

    // Check front wall — need to stop and decide turn
    if (front) {
        *vL = 0; *vR = 0;

        // Decide turn direction using wall follow rule
        float angle_deg = 0;
        if (s_wall_follow_side < 0) {
            // Left-hand rule: prefer left > front(already blocked) > right > U-turn
            if (!left)       angle_deg = -90.0f;  // turn left
            else if (!right) angle_deg =  90.0f;  // turn right
            else             angle_deg = 180.0f;  // U-turn
        } else {
            // Right-hand rule: prefer right > front > left > U-turn
            if (!right)      angle_deg =  90.0f;  // turn right
            else if (!left)  angle_deg = -90.0f;  // turn left
            else             angle_deg = 180.0f;  // U-turn
        }

        // Plan turn
        pid_reset(&s_heading_pid);
        pid_set_gains(&s_heading_pid,
                      PD_ROT_KP_X100 / 100.0f, 0,
                      PD_ROT_KD_X100 / 100.0f,
                      PD_ROT_MAX_MV);
        s_actual_dist = 0;
        profile_plan_turn(&s_profile, angle_deg,
                          (float)TURN_VEL_DPS, (float)TURN_ACCEL_DPS2, (float)TURN_ACCEL_DPS2);
        s_wf_phase = WF_TURNING;
        return true;
    }

    // Also check: if followed wall disappears (opening), turn into it
    if (s_wall_follow_side < 0 && !left) {
        // Left-hand rule: left wall gone = opening, turn left
        *vL = 0; *vR = 0;
        pid_reset(&s_heading_pid);
        pid_set_gains(&s_heading_pid,
                      PD_ROT_KP_X100 / 100.0f, 0,
                      PD_ROT_KD_X100 / 100.0f,
                      PD_ROT_MAX_MV);
        s_actual_dist = 0;
        profile_plan_turn(&s_profile, -90.0f,
                          (float)TURN_VEL_DPS, (float)TURN_ACCEL_DPS2, (float)TURN_ACCEL_DPS2);
        s_wf_phase = WF_TURNING;
        return true;
    }
    if (s_wall_follow_side > 0 && !right) {
        // Right-hand rule: right wall gone = opening, turn right
        *vL = 0; *vR = 0;
        pid_reset(&s_heading_pid);
        pid_set_gains(&s_heading_pid,
                      PD_ROT_KP_X100 / 100.0f, 0,
                      PD_ROT_KD_X100 / 100.0f,
                      PD_ROT_MAX_MV);
        s_actual_dist = 0;
        profile_plan_turn(&s_profile, 90.0f,
                          (float)TURN_VEL_DPS, (float)TURN_ACCEL_DPS2, (float)TURN_ACCEL_DPS2);
        s_wf_phase = WF_TURNING;
        return true;
    }

    // Normal drive with wall centering
    float target_vel = (float)WALL_FOLLOW_VEL_MMPS;
    float actual_vel = (float)(velL_mmps + velR_mmps) * 0.5f;
    float vel_corr = pid_tick(&s_vel_pid, target_vel - actual_vel);
    g_v_target_mmps = (int16_t)target_vel;

    float wall_err = wall_center_error();
    float wall_corr = pid_tick(&s_wall_pid, wall_err);
    float steer_corr = pid_tick(&s_heading_pid,
                                 s_target_heading - s_actual_heading) + wall_corr;

    float ff_L = (float)ff_voltage_L(target_vel, 0);
    float ff_R = (float)ff_voltage_R(target_vel, 0);

    *vL = clamp_i16(ff_L + vel_corr - steer_corr + s_steer_trim, -VCMD_MAX_MV, VCMD_MAX_MV);
    *vR = clamp_i16(ff_R + vel_corr + steer_corr - s_steer_trim, -VCMD_MAX_MV, VCMD_MAX_MV);
    return true;
}

// ═══════════════════════════════════════════════════════════════════════════════
// Continuous mode: start multi-cell drive
// ═══════════════════════════════════════════════════════════════════════════════
static void start_drive_continuous(void) {
    ensure_init();
    pid_reset(&s_vel_pid);
    pid_reset(&s_heading_pid);

    pid_set_gains(&s_heading_pid,
                  PD_STEER_KP_X100 / 100.0f,
                  PD_STEER_KI_X100 / 100.0f,
                  PD_STEER_KD_X100 / 100.0f,
                  PD_STEER_MAX_MV);

    s_cont_vel         = 0;
    s_cont_dist_cell   = 0;
    s_cont_stopping    = false;
    s_cont_heading_acc = 0;
    s_cont_timeout     = 0;

    s_state = SEARCH_DRIVING_MULTI;
}

// ═══════════════════════════════════════════════════════════════════════════════
// Continuous mode tick
// ═══════════════════════════════════════════════════════════════════════════════
static bool continuous_tick(int16_t* vL, int16_t* vR,
                            int16_t velL_mmps, int16_t velR_mmps,
                            float gyroZ_dps) {

    s_cont_timeout++;
    if (s_cont_timeout > 60000) {
        *vL = 0; *vR = 0;
        s_dbg_error_code = 2;  // timeout
        s_state = SEARCH_ERROR;
        return false;
    }

    float actual_vel = (float)(velL_mmps + velR_mmps) * 0.5f;
    float dist_step  = fabsf(actual_vel) * DT;
    s_cont_dist_cell += dist_step;
    s_cont_heading_acc += gyroZ_dps * (float)GYRO_Z_SIGN * DT;

    // ── Cell boundary crossing ──
    if (s_cont_dist_cell >= CELL_SIZE_MM) {
        s_cont_dist_cell -= CELL_SIZE_MM;

        uint8_t nx, ny;
        if (maze_step(s_pose.x, s_pose.y, s_pose.heading, &nx, &ny)) {
            s_pose.x = nx;
            s_pose.y = ny;
        }
        s_pose.steps++;

        read_and_update_walls();

        if (!s_return_mode && maze_is_goal(&s_maze, s_pose.x, s_pose.y)) {
            s_cont_stopping = true;
        } else if (s_return_mode && s_pose.x == s_start_x && s_pose.y == s_start_y) {
            s_cont_stopping = true;
        } else if (!s_cont_stopping) {
            if (!can_continue_straight()) {
                s_cont_stopping = true;
            }
        }

        pid_reset(&s_heading_pid);
        s_cont_heading_acc = 0;
    }

    // ── Velocity planning ──
    float target_vel;
    float target_acc;
    float cruise = (float)SEARCH_VEL_MMPS;
    float accel  = (float)SEARCH_ACCEL_MMPS2;
    float decel  = (float)SEARCH_DECEL_MMPS2;

    if (s_cont_stopping) {
        float remain = CELL_SIZE_MM - s_cont_dist_cell;
        if (remain < 1.0f) remain = 1.0f;

        float v_stop = sqrtf(2.0f * decel * remain);

        if (s_cont_vel <= 5.0f && remain < 5.0f) {
            *vL = 0; *vR = 0;
            s_cont_vel = 0;

            if (!s_return_mode && maze_is_goal(&s_maze, s_pose.x, s_pose.y)) {
                s_state = SEARCH_GOAL_REACHED;
            } else if (s_return_mode && s_pose.x == s_start_x && s_pose.y == s_start_y) {
                s_state = SEARCH_DONE;
            } else {
                s_state = SEARCH_DECIDING;
            }
            return (s_state != SEARCH_DONE && s_state != SEARCH_ERROR);
        }

        target_vel = fminf(s_cont_vel, v_stop);
        target_acc = -decel;
        s_cont_vel = fmaxf(0.0f, s_cont_vel - decel * DT);

    } else if (s_cont_vel < cruise) {
        s_cont_vel += accel * DT;
        if (s_cont_vel > cruise) s_cont_vel = cruise;
        target_vel = s_cont_vel;
        target_acc = accel;

    } else {
        s_cont_vel = cruise;
        target_vel = cruise;
        target_acc = 0;
    }

    float vel_corr = pid_tick(&s_vel_pid, target_vel - actual_vel);
    // ★ Wall centering in continuous mode too
    float wall_err = wall_center_error();
    float wall_corr = pid_tick(&s_wall_pid, wall_err);
    float steer_corr = pid_tick(&s_heading_pid, 0 - s_cont_heading_acc) + wall_corr;
    g_v_target_mmps = (int16_t)target_vel;  // expose for telemetry

    float ff_L = (float)ff_voltage_L(target_vel, target_acc);
    float ff_R = (float)ff_voltage_R(target_vel, target_acc);

    *vL = clamp_i16(ff_L + vel_corr - steer_corr + s_steer_trim, -VCMD_MAX_MV, VCMD_MAX_MV);
    *vR = clamp_i16(ff_R + vel_corr + steer_corr - s_steer_trim, -VCMD_MAX_MV, VCMD_MAX_MV);

    return true;
}

// ═══════════════════════════════════════════════════════════════════════════════
// Search FSM: decide next action
// ═══════════════════════════════════════════════════════════════════════════════
static void search_decide(void) {
    read_and_update_walls();

    if (s_return_mode) {
        maze_flood_target(&s_maze, s_start_x, s_start_y);
    } else {
        maze_flood(&s_maze);
    }

    if (!s_return_mode && maze_is_goal(&s_maze, s_pose.x, s_pose.y)) {
        s_state = SEARCH_GOAL_REACHED;
        return;
    }
    if (s_return_mode && s_pose.x == s_start_x && s_pose.y == s_start_y) {
        s_state = SEARCH_DONE;
        return;
    }

    int8_t best = maze_best_dir(&s_maze, s_pose.x, s_pose.y);
    if (best < 0) {
        s_dbg_error_code = 1;  // stuck — no open path
        s_state = SEARCH_ERROR;
        return;
    }

    int8_t turn = maze_turn_needed(s_pose.heading, (uint8_t)best);

    if (turn != 0) {
        s_state = SEARCH_TURNING;
        start_turn(turn);
        s_pose.heading = (uint8_t)best;
    } else {
        if (s_mode == SEARCH_MODE_CONTINUOUS) {
            start_drive_continuous();
        } else {
            s_state = SEARCH_DRIVING;
            start_drive_single();
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
// Public: handle test command
// ═══════════════════════════════════════════════════════════════════════════════
// ★ Apply goal/start config from command payload (embedded in RUN/STEP/RESET)
// payload[2] = 0xCF magic marker ("ConFig present")
// payload[3] = start_x, [4] = start_y, [5] = start_heading (0=N,1=E,2=S,3=W)
// payload[6] = goal_x_min, [7] = goal_y_min, [8] = goal_x_max, [9] = goal_y_max
static void apply_inline_config(const uint8_t* payload) {
    if (payload[2] != 0xCF) return;  // no config embedded
    uint8_t sx = payload[3], sy = payload[4], sh = payload[5];
    uint8_t gxn = payload[6], gyn = payload[7], gxx = payload[8], gyx = payload[9];
    if (sx < MAZE_SIZE) s_start_x = sx;
    if (sy < MAZE_SIZE) s_start_y = sy;
    s_start_heading_deg = (sh & 3) * 90;
    if (gxn < MAZE_SIZE) s_goal_x_min = gxn;
    if (gyn < MAZE_SIZE) s_goal_y_min = gyn;
    if (gxx < MAZE_SIZE && gxx >= gxn) s_goal_x_max = gxx;
    if (gyx < MAZE_SIZE && gyx >= gyn) s_goal_y_max = gyx;
}

bool lab5_handle_test(const CommandRequest* cmd, CommandResult* result,
                      int16_t* vCmdL_mV, int16_t* vCmdR_mV) {
    uint8_t test_id = cmd->payload[0];
    memset(result, 0, sizeof(*result));
    result->opcode = cmd->opcode;
    result->req_id = cmd->req_id;
    result->status = RSP_OK;

    switch (test_id) {
    case TEST_SEARCH_RUN: {
        uint8_t mode_byte = cmd->payload[1];
        s_mode = (mode_byte == 1) ? SEARCH_MODE_CONTINUOUS : SEARCH_MODE_STOP_GO;
        apply_inline_config(cmd->payload);  // ★ apply goal/start from GUI
        full_reset();       // ★ Always fresh start for RUN
        s_continuous = true;
        s_return_mode = false;
        s_sub = SUB_IDLE;
        s_tick_count = 0;
        s_state = SEARCH_WARMUP;   // ★ warmup before first decision
        *vCmdL_mV = 0; *vCmdR_mV = 0;
        return true;
    }
    case TEST_SEARCH_STEP: {
        // ★ KEY FIX: Only full_reset on very first STEP or after explicit RESET.
        apply_inline_config(cmd->payload);  // ★ apply goal/start from GUI
        if (!s_maze_inited || s_state == SEARCH_ERROR || s_state == SEARCH_DONE) {
            full_reset();
            s_tick_count = 0;
            s_state = SEARCH_WARMUP;   // ★ warmup on first step
        } else {
            s_state = SEARCH_DECIDING; // ★ continue from current pose
            // ★ Update maze goal in case GUI changed it between steps
            s_maze.goal_x_min = s_goal_x_min;
            s_maze.goal_y_min = s_goal_y_min;
            s_maze.goal_x_max = s_goal_x_max;
            s_maze.goal_y_max = s_goal_y_max;
        }
        s_mode = SEARCH_MODE_STOP_GO;
        s_continuous = false;
        s_return_mode = false;
        s_sub = SUB_IDLE;
        *vCmdL_mV = 0; *vCmdR_mV = 0;
        return true;
    }
    case TEST_SEARCH_RETURN: {
        s_continuous = true;
        s_return_mode = true;
        s_state = SEARCH_DECIDING;
        s_sub = SUB_IDLE;
        *vCmdL_mV = 0; *vCmdR_mV = 0;
        return true;
    }
    case TEST_SEARCH_RESET: {
        apply_inline_config(cmd->payload);  // ★ apply goal/start from GUI
        full_reset();
        s_maze_inited = false;   // ★ next STEP will do full_reset + warmup
        s_state = SEARCH_IDLE;
        s_sub = SUB_IDLE;
        *vCmdL_mV = 0; *vCmdR_mV = 0;
        return true;
    }
    case TEST_SEARCH_READ_WALLS: {
        // ★ One-shot: sample all sensors NOW and run through hysteresis
        wall_detect_tick_all();

        // Use debounced states from WallDetector
        bool det_l  = s_wd[WD_IDX_L].state;
        bool det_fl = s_wd[WD_IDX_FL].state;
        bool det_fr = s_wd[WD_IDX_FR].state;
        bool det_r  = s_wd[WD_IDX_R].state;
        bool det_front = wall_detect_get_front();

        // RSP payload (13 bytes):
        //   [0-1] left_mm   [2-3] FL_mm  [4-5] FR_mm  [6-7] right_mm
        //   [8]   flags: b0=L b1=FL b2=FR b3=R b4=front_combined
        //   [9]   heading  [10] x  [11] y  [12] debounce_info
        uint8_t* rp = result->rsp_payload;
        int16_t lv = s_wd[WD_IDX_L].last_mm;
        int16_t flv = s_wd[WD_IDX_FL].last_mm;
        int16_t frv = s_wd[WD_IDX_FR].last_mm;
        int16_t rv = s_wd[WD_IDX_R].last_mm;
        rp[0] = (uint8_t)(lv & 0xFF);    rp[1] = (uint8_t)(lv >> 8);
        rp[2] = (uint8_t)(flv & 0xFF);   rp[3] = (uint8_t)(flv >> 8);
        rp[4] = (uint8_t)(frv & 0xFF);   rp[5] = (uint8_t)(frv >> 8);
        rp[6] = (uint8_t)(rv & 0xFF);    rp[7] = (uint8_t)(rv >> 8);
        rp[8] = (det_l ? 1 : 0) | (det_fl ? 2 : 0) | (det_fr ? 4 : 0) |
                (det_r ? 8 : 0) | (det_front ? 16 : 0);
        rp[9]  = s_pose.heading;
        rp[10] = s_pose.x;
        rp[11] = s_pose.y;
        // Pack debounce counters (2 bits each, showing stability)
        rp[12] = (s_wd[WD_IDX_L].counter & 0x03) |
                 ((s_wd[WD_IDX_FL].counter & 0x03) << 2) |
                 ((s_wd[WD_IDX_FR].counter & 0x03) << 4) |
                 ((s_wd[WD_IDX_R].counter & 0x03) << 6);
        result->rsp_payload_len = 13;
        *vCmdL_mV = 0; *vCmdR_mV = 0;
        return true;
    }
    case TEST_SEARCH_QUERY: {
        // ★ GUI polls this at 2-5Hz to track robot pose + state
        // RSP payload (13 bytes):
        //   [0] search_state  [1] mode  [2] x  [3] y  [4] heading
        //   [5-6] steps(u16)  [7] error_code
        //   [8] walls_at_cell (4-bit: NESW)  [9] visited_count
        //   [10] goal_x_min | (goal_x_max << 4)
        //   [11] goal_y_min | (goal_y_max << 4)
        //   [12] start_x | (start_y << 4)
        uint8_t* rp = result->rsp_payload;
        rp[0] = (uint8_t)s_state;
        rp[1] = (uint8_t)s_mode;
        rp[2] = s_pose.x;
        rp[3] = s_pose.y;
        rp[4] = s_pose.heading;
        rp[5] = (uint8_t)(s_pose.steps & 0xFF);
        rp[6] = (uint8_t)(s_pose.steps >> 8);
        rp[7] = s_dbg_error_code;
        rp[8] = s_maze.walls[s_pose.y][s_pose.x] & CELL_WALLS_MASK;
        // Count visited cells
        uint8_t vc = 0;
        for (uint8_t yy = 0; yy < MAZE_SIZE; yy++)
            for (uint8_t xx = 0; xx < MAZE_SIZE; xx++)
                if (s_maze.walls[yy][xx] & CELL_VISITED) vc++;
        rp[9] = vc;
        rp[10] = (s_goal_x_min & 0x0F) | ((s_goal_x_max & 0x0F) << 4);
        rp[11] = (s_goal_y_min & 0x0F) | ((s_goal_y_max & 0x0F) << 4);
        rp[12] = (s_start_x & 0x0F) | ((s_start_y & 0x0F) << 4);
        result->rsp_payload_len = 13;
        *vCmdL_mV = 0; *vCmdR_mV = 0;
        return true;
    }
    case TEST_SEARCH_MAZE_ROW: {
        // ★ Returns wall data for one row (16 cells packed)
        // Request: payload[1] = row (0-15)
        // RSP payload (13 bytes):
        //   [0]   row number
        //   [1-8] walls for 16 cells (4 bits each, 2 cells/byte)
        //         byte[1] = cell[0] low nibble | cell[1] high nibble
        //   [9-10] visited bitmask (16 bits, cell 0 = bit 0)
        //   [11-12] reserved
        uint8_t row = cmd->payload[1];
        if (row >= MAZE_SIZE) row = 0;
        uint8_t* rp = result->rsp_payload;
        rp[0] = row;
        for (uint8_t i = 0; i < 8; i++) {
            uint8_t c0 = s_maze.walls[row][i * 2] & CELL_WALLS_MASK;
            uint8_t c1 = s_maze.walls[row][i * 2 + 1] & CELL_WALLS_MASK;
            rp[1 + i] = c0 | (c1 << 4);
        }
        uint16_t vis = 0;
        for (uint8_t xx = 0; xx < MAZE_SIZE; xx++) {
            if (s_maze.walls[row][xx] & CELL_VISITED) vis |= (1 << xx);
        }
        rp[9]  = (uint8_t)(vis & 0xFF);
        rp[10] = (uint8_t)(vis >> 8);
        rp[11] = 0; rp[12] = 0;
        result->rsp_payload_len = 13;
        *vCmdL_mV = 0; *vCmdR_mV = 0;
        return true;
    }
    case 0x57:   // TEST_WALL_FOLLOW_L — follow left wall
    case 0x58: { // TEST_WALL_FOLLOW_R — follow right wall
        ensure_init();
        pid_reset(&s_vel_pid);
        pid_reset(&s_heading_pid);
        pid_reset(&s_wall_pid);

        pid_set_gains(&s_heading_pid,
                      PD_STEER_KP_X100 / 100.0f,
                      PD_STEER_KI_X100 / 100.0f,
                      PD_STEER_KD_X100 / 100.0f,
                      PD_STEER_MAX_MV);

        s_actual_heading = 0;
        s_target_heading = 0;
        s_actual_dist = 0;
        s_tick_count = 0;
        s_wall_follow_side = (test_id == 0x57) ? -1 : +1;  // -1=left, +1=right
        s_sub = SUB_WALL_FOLLOW;
        s_state = SEARCH_DRIVING;
        *vCmdL_mV = 0; *vCmdR_mV = 0;
        return true;
    }
    case 0x59: { // TEST_SEARCH_SET_CONFIG — set goal + start position from GUI
        // payload[1] = start_x, [2] = start_y, [3] = start_heading (0=N,1=E,2=S,3=W → ×90°)
        // payload[4] = goal_x_min, [5] = goal_y_min, [6] = goal_x_max, [7] = goal_y_max
        uint8_t sx = cmd->payload[1];
        uint8_t sy = cmd->payload[2];
        uint8_t sh = cmd->payload[3];
        uint8_t gxn = cmd->payload[4];
        uint8_t gyn = cmd->payload[5];
        uint8_t gxx = cmd->payload[6];
        uint8_t gyx = cmd->payload[7];

        // Validate ranges
        if (sx < MAZE_SIZE) s_start_x = sx;
        if (sy < MAZE_SIZE) s_start_y = sy;
        s_start_heading_deg = (sh & 3) * 90;
        if (gxn < MAZE_SIZE) s_goal_x_min = gxn;
        if (gyn < MAZE_SIZE) s_goal_y_min = gyn;
        if (gxx < MAZE_SIZE && gxx >= gxn) s_goal_x_max = gxx;
        if (gyx < MAZE_SIZE && gyx >= gyn) s_goal_y_max = gyx;

        // RSP: echo back the applied config
        uint8_t* rp = result->rsp_payload;
        rp[0] = s_start_x; rp[1] = s_start_y;
        rp[2] = s_start_heading_deg / 90;
        rp[3] = s_goal_x_min; rp[4] = s_goal_y_min;
        rp[5] = s_goal_x_max; rp[6] = s_goal_y_max;
        result->rsp_payload_len = 7;
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
bool lab5_tick(int16_t* vCmdL_mV, int16_t* vCmdR_mV,
               int16_t velL_mmps, int16_t velR_mmps,
               float gyroZ_dps) {

    // Terminal states
    if (s_state == SEARCH_IDLE || s_state == SEARCH_DONE ||
        s_state == SEARCH_ERROR) {
        *vCmdL_mV = 0; *vCmdR_mV = 0;
        return false;
    }

    // ★ Sample wall sensors at 1kHz — hysteresis + debounce run continuously
    //   This ensures wall states are stable by the time read_and_update_walls() uses them
    wall_detect_tick_all();

    // ★ WARMUP: let wall detectors run for N ticks before first decision
    //   Sensors need time after reset to produce valid debounced readings.
    if (s_state == SEARCH_WARMUP) {
        s_tick_count++;
        *vCmdL_mV = 0; *vCmdR_mV = 0;
        if (s_tick_count >= WARMUP_TICKS) {
            s_state = SEARCH_DECIDING;
        }
        return true;
    }

    if (s_state == SEARCH_GOAL_REACHED) {
        *vCmdL_mV = 0; *vCmdR_mV = 0;
        if (s_continuous) {
            s_return_mode = true;
            s_state = SEARCH_DECIDING;
            return true;
        }
        return false;
    }

    // DECIDING
    if (s_state == SEARCH_DECIDING) {
        search_decide();
        if (s_state != SEARCH_TURNING && s_state != SEARCH_DRIVING &&
            s_state != SEARCH_DRIVING_MULTI) {
            *vCmdL_mV = 0; *vCmdR_mV = 0;
            return true;
        }
    }

    // TURNING
    if (s_state == SEARCH_TURNING) {
        bool running = sub_motion_tick(vCmdL_mV, vCmdR_mV,
                                       velL_mmps, velR_mmps, gyroZ_dps);
        if (!running) {
            // ═════════════════════════════════════════════════════════
            // ★ LAYER 3: POST-TURN RECHECK
            //   After turning, re-read front sensors before driving.
            //   If wall detected → side sensor was wrong about open path.
            //   Update maze with corrected wall info → re-decide.
            //
            //   Geometry: at cell center, front sensor → boundary = 26mm
            //   If wall at boundary: sensor reads 26mm → < 40mm → BLOCKED
            //   If open:             sensor reads ≥100mm → > 40mm → CLEAR
            // ═════════════════════════════════════════════════════════
#if FRONT_RECHECK_ENABLE
            {
                int16_t fl = s_wd[WD_IDX_FL].last_mm;
                int16_t fr = s_wd[WD_IDX_FR].last_mm;
                int16_t front = min_i16(fl, fr);

                if (front > 0 && front < FRONT_RECHECK_MM) {
                    // Wall detected after turn! Side sensor was wrong.
                    // → Force-update maze with wall in current heading
                    safe_update_wall(&s_maze, s_pose.x, s_pose.y,
                                     s_pose.heading, true);
                    // → Re-flood and re-decide (don't drive into wall)
                    s_state = SEARCH_DECIDING;
                    *vCmdL_mV = 0; *vCmdR_mV = 0;
                    return true;
                }
            }
#endif
            // No wall → proceed to drive
            if (s_mode == SEARCH_MODE_CONTINUOUS) {
                start_drive_continuous();
            } else {
                s_state = SEARCH_DRIVING;
                start_drive_single();
            }
        }
        return true;
    }

    // DRIVING — Stop-and-Go
    if (s_state == SEARCH_DRIVING) {
        // Wall follow mode has its own tick
        if (s_sub == SUB_WALL_FOLLOW) {
            bool running = wall_follow_tick(vCmdL_mV, vCmdR_mV,
                                             velL_mmps, velR_mmps, gyroZ_dps);
            return running;
        }
        bool running = sub_motion_tick(vCmdL_mV, vCmdR_mV,
                                       velL_mmps, velR_mmps, gyroZ_dps);
        if (!running) {
            uint8_t nx, ny;
            if (maze_step(s_pose.x, s_pose.y, s_pose.heading, &nx, &ny)) {
                s_pose.x = nx;
                s_pose.y = ny;
            }
            s_pose.steps++;

            if (s_continuous) {
                s_state = SEARCH_DECIDING;
            } else {
                s_state = SEARCH_IDLE;
            }
        }
        return true;
    }

    // DRIVING_MULTI — Continuous
    if (s_state == SEARCH_DRIVING_MULTI) {
        bool running = continuous_tick(vCmdL_mV, vCmdR_mV,
                                       velL_mmps, velR_mmps, gyroZ_dps);
        if (!running) {
            *vCmdL_mV = 0; *vCmdR_mV = 0;
        }
        return (s_state != SEARCH_IDLE && s_state != SEARCH_DONE &&
                s_state != SEARCH_ERROR);
    }

    *vCmdL_mV = 0; *vCmdR_mV = 0;
    return true;
}

// ═══════════════════════════════════════════════════════════════════════════════
// Public: stop / query
// ═══════════════════════════════════════════════════════════════════════════════

void lab5_stop(void) {
    s_state = SEARCH_IDLE;
    s_sub = SUB_IDLE;
    s_cont_vel = 0;
    profile_reset(&s_profile);
}

bool lab5_is_running(void) {
    return s_state != SEARCH_IDLE && s_state != SEARCH_DONE &&
           s_state != SEARCH_ERROR;
}

SearchState    lab5_get_state(void) { return s_state; }
SearchMode     lab5_get_mode(void)  { return s_mode; }
const RobotPose* lab5_get_pose(void) { return &s_pose; }
const Maze*    lab5_get_maze(void)   { return &s_maze; }
