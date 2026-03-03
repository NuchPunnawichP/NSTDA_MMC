// ═══════════════════════════════════════════════════════════════════════════════
//  lab1_hw_check.h — Lab 1: Hardware Check Test Handlers
//
//  Layer: Labs
//  Dependencies: protocol.h, HALs
//
//  Tests:
//    0x01 MOTOR_SPIN     — spin motor(s) at given mV for duration
//    0x02 LED_SET        — set RGB LED color
//    0x03 BUZZER         — play tone
//    0x04 READ_SENSORS   — read wall sensors → RSP with 4× uint16 mm
//    0x05 READ_BATTERY   — read battery → RSP with mV + level
//    0x06 READ_IMU       — read IMU → RSP with gyro + accel (raw)
//    0x07 READ_BUTTONS   — read buttons + DIP → RSP
//
//  Motor spin uses a timed sequence: set vCmd for duration, then stop.
//  Managed by lab1_tick() called from step1kHz().
// ═══════════════════════════════════════════════════════════════════════════════
#ifndef LAB1_HW_CHECK_H
#define LAB1_HW_CHECK_H

#include <stdint.h>
#include <stdbool.h>
#include "protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

// ── Handle a RUN_TEST command for Lab 1 ─────────────────────────────────────
//  cmd: the full CommandRequest (payload[0]=test_id, [1..8]=params)
//  result: filled with status + optional RSP payload
//  vCmdL, vCmdR: output motor commands (mV) — caller applies to motors
//
//  Returns true if test was recognized (even if error).
bool lab1_handle_test(const CommandRequest* cmd, CommandResult* result,
                      int16_t* vCmdL_mV, int16_t* vCmdR_mV);

// ── Called every 1kHz tick from ControlTask ──────────────────────────────────
//  Manages timed motor tests (countdown → auto-stop).
//  Returns true if a motor test is actively running.
//  vCmdL, vCmdR: output — current motor command for this tick.
bool lab1_tick(int16_t* vCmdL_mV, int16_t* vCmdR_mV);

// ── Stop any running test ───────────────────────────────────────────────────
void lab1_stop(void);

// ── Is a test currently running? ────────────────────────────────────────────
bool lab1_is_running(void);

#ifdef __cplusplus
}
#endif

#endif // LAB1_HW_CHECK_H
