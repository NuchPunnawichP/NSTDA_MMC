// ═══════════════════════════════════════════════════════════════════════════════
//  lab6_speedrun.h — Lab 6: Speed Run through Known Maze
//
//  Prereq: Lab 5 must have completed a search → maze is populated
//
//  Concept:
//    1. Extract shortest path from Lab 5 maze (flood → trace back)
//    2. Run through the path at configurable speed levels
//    3. Uses continuous motion (no stop between straight cells)
//
//  Speed levels:
//    0 = cautious (same as search speed, for testing)
//    1 = moderate (350 mm/s)
//    2 = fast (500 mm/s)
//    3 = fastest (700 mm/s, competition)
//
//  Tests (via BLE CMD_RUN_TEST):
//    0x60 SPEEDRUN_START  — payload[1]=speed_level (0-3)
//    0x61 SPEEDRUN_STOP   — abort
//    0x62 SPEEDRUN_QUERY  — read-only: state + progress
// ═══════════════════════════════════════════════════════════════════════════════
#ifndef LAB6_SPEEDRUN_H
#define LAB6_SPEEDRUN_H

#include <stdint.h>
#include <stdbool.h>
#include "protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

// ── Speed run state ──────────────────────────────────────────────────────────
typedef enum {
    SPEEDRUN_IDLE = 0,
    SPEEDRUN_PLANNING,      // extracting path from maze
    SPEEDRUN_RUNNING,       // following path
    SPEEDRUN_TURNING,       // executing turn at waypoint
    SPEEDRUN_DONE,          // reached goal
    SPEEDRUN_ERROR,         // path not found or failure
} SpeedrunState;

// ── Max path length ──────────────────────────────────────────────────────────
#define SPEEDRUN_MAX_PATH   256     // worst case: visit every cell

// ── Public API ───────────────────────────────────────────────────────────────

bool lab6_handle_test(const CommandRequest* cmd, CommandResult* result,
                      int16_t* vCmdL_mV, int16_t* vCmdR_mV);

bool lab6_tick(int16_t* vCmdL_mV, int16_t* vCmdR_mV,
               int16_t velL_mmps, int16_t velR_mmps,
               float gyroZ_dps);

void lab6_stop(void);
bool lab6_is_running(void);

SpeedrunState lab6_get_state(void);
uint16_t      lab6_get_path_len(void);
uint16_t      lab6_get_progress(void);

#ifdef __cplusplus
}
#endif

#endif // LAB6_SPEEDRUN_H
