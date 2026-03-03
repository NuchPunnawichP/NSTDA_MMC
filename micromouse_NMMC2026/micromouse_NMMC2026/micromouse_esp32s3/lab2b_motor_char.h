// ═══════════════════════════════════════════════════════════════════════════════
//  lab2b_motor_char.h — Lab 2b: Motor Characterization (Feedforward)
//
//  Tests:
//    0x20 RAMP_UP        — sweep mV from start→end in steps, FAST streams data
//    0x21 STEP_RESPONSE  — apply constant mV for duration, FAST streams data
//    0x22 FIND_BIAS      — auto-find min mV to start moving → RSP biasL, biasR
//
//  Design:
//    All tests use existing FAST stream (50Hz). GUI records packets during test.
//    ESP32 controls voltage pattern + auto-stops. GUI does analysis + plotting.
// ═══════════════════════════════════════════════════════════════════════════════
#ifndef LAB2B_MOTOR_CHAR_H
#define LAB2B_MOTOR_CHAR_H

#include <stdint.h>
#include <stdbool.h>
#include "protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

// Handle Lab 2b test. Sets vCmdL/R if motor test started.
bool lab2b_handle_test(const CommandRequest* cmd, CommandResult* result,
                       int16_t* vCmdL_mV, int16_t* vCmdR_mV);

// Called every 1kHz tick — manages timed voltage patterns.
// Returns true if actively running. Outputs current vCmd.
bool lab2b_tick(int16_t* vCmdL_mV, int16_t* vCmdR_mV);

void lab2b_stop(void);
bool lab2b_is_running(void);

// Get FIND_BIAS result after test completes
void lab2b_get_bias_result(int16_t* biasL_mV, int16_t* biasR_mV);

#ifdef __cplusplus
}
#endif

#endif // LAB2B_MOTOR_CHAR_H
