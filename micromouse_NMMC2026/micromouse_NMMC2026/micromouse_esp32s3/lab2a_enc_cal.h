// ═══════════════════════════════════════════════════════════════════════════════
//  lab2a_enc_cal.h — Lab 2a: Encoder Calibration
//
//  Tests:
//    0x10 ENC_MONITOR   — read absolute counts L/R → RSP (poll from GUI)
//    0x11 ENC_RESET     — reset accumulators to 0
//
//  Procedure (student):
//    1. Reset encoders
//    2. Push wheel exactly one revolution (tape mark)
//    3. Read counts → CPR
//    4. mm_per_count = (π × wheel_dia_mm) / CPR
// ═══════════════════════════════════════════════════════════════════════════════
#ifndef LAB2A_ENC_CAL_H
#define LAB2A_ENC_CAL_H

#include <stdint.h>
#include <stdbool.h>
#include "protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

// Handle Lab 2a test. Returns true if test_id recognized.
bool lab2a_handle_test(const CommandRequest* cmd, CommandResult* result);

#ifdef __cplusplus
}
#endif

#endif // LAB2A_ENC_CAL_H
