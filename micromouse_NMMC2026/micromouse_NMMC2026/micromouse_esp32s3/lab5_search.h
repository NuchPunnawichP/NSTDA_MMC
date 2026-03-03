// ═══════════════════════════════════════════════════════════════════════════════
//  lab5_search.h — Lab 5: Maze Search (Flood Fill)
//
//  Two search modes:
//    STOP_AND_GO:  stop at every cell → read walls → decide → move 1 cell
//                  Safe, easy to debug. Good for first runs.
//
//    CONTINUOUS:   read walls while moving. Only stop before turns.
//                  Multi-cell straights without accel/decel between cells.
//                  2-3× faster than stop-and-go.
//
//  Tests (via BLE CMD_TEST_RUN):
//    0x50 SEARCH_START   — full search (payload[1]: 0=stop-go, 1=continuous)
//    0x51 SEARCH_STEP    — single step (one cell at a time)
//    0x52 SEARCH_RETURN  — return to (0,0) using known shortest path
//    0x53 SEARCH_RESET   — reset maze + position
// ═══════════════════════════════════════════════════════════════════════════════
#ifndef LAB5_SEARCH_H
#define LAB5_SEARCH_H

#include <stdint.h>
#include <stdbool.h>
#include "protocol.h"
#include "maze.h"

#ifdef __cplusplus
extern "C" {
#endif

// ── Search mode ──────────────────────────────────────────────────────────────
typedef enum {
    SEARCH_MODE_STOP_GO    = 0,     // stop at every cell (safe, debug)
    SEARCH_MODE_CONTINUOUS = 1,     // multi-cell straights, stop only for turns
} SearchMode;

// ── Search state (visible to GUI) ────────────────────────────────────────────
typedef enum {
    SEARCH_IDLE = 0,
    SEARCH_WARMUP,              // sensor warmup before first decision
    SEARCH_READING_WALLS,       // reading sensors at current cell
    SEARCH_DECIDING,            // flood fill + pick direction
    SEARCH_TURNING,             // executing turn
    SEARCH_DRIVING,             // driving forward (1 cell, stop-and-go)
    SEARCH_DRIVING_MULTI,       // driving forward (multi-cell continuous)
    SEARCH_GOAL_REACHED,        // arrived at goal
    SEARCH_RETURNING,           // returning to start
    SEARCH_DONE,                // back at start
    SEARCH_ERROR,               // stuck or failure
} SearchState;

// ── Robot pose ───────────────────────────────────────────────────────────────
typedef struct {
    uint8_t x, y;               // grid coordinates
    uint8_t heading;            // DIR_N/E/S/W
    uint16_t steps;             // total steps taken
} RobotPose;

// ── Handle Lab 5 test command ────────────────────────────────────────────────
bool lab5_handle_test(const CommandRequest* cmd, CommandResult* result,
                      int16_t* vCmdL_mV, int16_t* vCmdR_mV);

// ── Tick @ 1kHz ──────────────────────────────────────────────────────────────
bool lab5_tick(int16_t* vCmdL_mV, int16_t* vCmdR_mV,
               int16_t velL_mmps, int16_t velR_mmps,
               float gyroZ_dps);

void lab5_stop(void);
bool lab5_is_running(void);

// ── Query (for telemetry / GUI) ──────────────────────────────────────────────
SearchState    lab5_get_state(void);
SearchMode     lab5_get_mode(void);
const RobotPose* lab5_get_pose(void);
const Maze*    lab5_get_maze(void);

// ── Debug: wall sensor readings (last read_and_update_walls) ─────────────────
//  index: 0=Left, 1=FrontL, 2=FrontR, 3=Right
int16_t  lab5_dbg_wall_mm(uint8_t index);       // raw distance mm
bool     lab5_dbg_wall_det(uint8_t index);       // detected as wall?
uint8_t  lab5_dbg_error_code(void);              // 0=ok, 1=stuck, 2=timeout

#ifdef __cplusplus
}
#endif

#endif // LAB5_SEARCH_H
