// ═══════════════════════════════════════════════════════════════════════════════
//  competition.h — Standalone Competition Controller
//
//  ★ ไม่ต้องสร้างโปรเจคใหม่ — ใช้ร่วมกับ BLE debug ได้
//
//  DIP switch → เลือก mode
//  Push button → เริ่ม/หยุด
//  LED → แสดงสถานะ
//
//  วิธีใช้:
//    1. ตั้ง DIP switch เลือก mode (ดู COMP_MODE_*)
//    2. กด button สั้น → ARM → กด button อีกครั้ง → เริ่มทำงาน
//    3. กด button ค้าง >2s → reset
//
//  DIP settings (4-bit):
//    0000 (0): BLE DEBUG (ค่า default ทำงานปกติผ่าน GUI)
//    0001 (1): Search stop-and-go
//    0010 (2): Search continuous
//    0011 (3): Speed run Lv.0 (ต้อง search ก่อน)
//    0100 (4): Speed run Lv.1
//    0101 (5): Speed run Lv.2
//    0110 (6): Speed run Lv.3
//    0111 (7): FULL COMPETITION (search → return → wait → speed run Lv.2)
//    1000 (8): Wall follow left (test)
//    1001 (9): Wall follow right (test)
// ═══════════════════════════════════════════════════════════════════════════════
#ifndef COMPETITION_H
#define COMPETITION_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// ── Competition modes (from DIP switch) ──────────────────────────────────────
#define COMP_MODE_BLE_DEBUG     0   // GUI mode (competition.cpp does nothing)
#define COMP_MODE_SEARCH_SG     1   // Search stop-and-go
#define COMP_MODE_SEARCH_CONT   2   // Search continuous
#define COMP_MODE_SPEED_LV0     3   // Speed run Level 0
#define COMP_MODE_SPEED_LV1     4   // Speed run Level 1
#define COMP_MODE_SPEED_LV2     5   // Speed run Level 2
#define COMP_MODE_SPEED_LV3     6   // Speed run Level 3
#define COMP_MODE_FULL_COMP     7   // Full: search → return → speed run
#define COMP_MODE_WALL_FOLLOW_L 8   // Wall follow left (test)
#define COMP_MODE_WALL_FOLLOW_R 9   // Wall follow right (test)

// ── Competition states ───────────────────────────────────────────────────────
typedef enum {
    COMP_IDLE = 0,          // LED off — waiting for button
    COMP_ARMED,             // LED blue — ready, press button to start
    COMP_SEARCHING,         // LED yellow — search in progress
    COMP_SEARCH_DONE,       // LED green blink — search complete
    COMP_RETURNING,         // LED cyan — returning to start
    COMP_WAIT_SPEEDRUN,     // LED white blink — waiting for button to speed run
    COMP_SPEEDRUN,          // LED magenta — speed run in progress
    COMP_DONE,              // LED green solid — all done
    COMP_ERROR,             // LED red — error occurred
} CompState;

// ── API ──────────────────────────────────────────────────────────────────────

// Call once at startup (reads DIP switches)
void competition_init(void);

// Call at 1kHz from control_task (alongside lab ticks)
// Returns true if competition mode is active (not BLE_DEBUG)
// and is currently controlling motors (sets vCmdL/R)
bool competition_tick(int16_t* vCmdL_mV, int16_t* vCmdR_mV,
                      int16_t velL_mmps, int16_t velR_mmps,
                      float gyroZ_dps);

// Is competition mode active? (DIP != 0)
bool competition_is_active(void);

// Get current state (for telemetry/debug)
CompState competition_get_state(void);
uint8_t   competition_get_mode(void);

#ifdef __cplusplus
}
#endif

#endif // COMPETITION_H
