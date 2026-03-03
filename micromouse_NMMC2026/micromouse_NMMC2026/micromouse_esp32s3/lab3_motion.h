// ═══════════════════════════════════════════════════════════════════════════════
//  lab3_motion.h — Lab 3: Motion Profile (Trapezoid)
//
//  Tests:
//    0x30 TRAP_FWD   — drive straight using trapezoidal velocity profile
//                      p1=distance_mm, p2=max_vel_mmps, p3=accel_mmps2
//    0x31 TRAP_TURN  — turn in place using trapezoidal angular velocity
//                      p1=angle_deg(+CW), p2=max_dps, p3=accel_dps2
//
//  Design:
//    - Profile generates target velocity each tick
//    - Feedforward converts target vel → motor voltage (per-wheel bias)
//    - Encoder feedback tracks actual distance for decel timing
//    - No PID yet — pure feedforward (Lab 4 adds PID correction)
// ═══════════════════════════════════════════════════════════════════════════════
#ifndef LAB3_MOTION_H
#define LAB3_MOTION_H

#include <stdint.h>
#include <stdbool.h>
#include "protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

// Handle Lab 3 test command. Returns true if test_id recognized.
bool lab3_handle_test(const CommandRequest* cmd, CommandResult* result,
                      int16_t* vCmdL_mV, int16_t* vCmdR_mV);

// Called every 1kHz tick. Returns true if running, sets motor voltages.
// encDeltaL/R: encoder counts since last tick (from hal_encoder_delta)
bool lab3_tick(int16_t* vCmdL_mV, int16_t* vCmdR_mV,
               int16_t encDeltaL, int16_t encDeltaR);

void lab3_stop(void);
bool lab3_is_running(void);

#ifdef __cplusplus
}
#endif

#endif // LAB3_MOTION_H
