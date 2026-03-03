// ═══════════════════════════════════════════════════════════════════════════════
//  lab4_pid.h — Lab 4: PID-Controlled Motion
//
//  Tests:
//    0x40 STRAIGHT     — PID straight: distance_mm, vel, accel
//    0x41 TURN_90      — PID 90° turn: direction (+1 CW, -1 CCW)
//    0x42 TURN_180     — PID 180° turn: direction
//    0x44 MOVE_CELLS   — PID straight N cells
//
//  Architecture:
//    Profile (target vel) + Feedforward (base voltage)
//    + Velocity PID (speed correction) + Heading PID (steer correction)
//
//    vL = FF_L(target) + vel_pid - heading_pid
//    vR = FF_R(target) + vel_pid + heading_pid
// ═══════════════════════════════════════════════════════════════════════════════
#ifndef LAB4_PID_H
#define LAB4_PID_H

#include <stdint.h>
#include <stdbool.h>
#include "protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

// Handle Lab 4 test command
bool lab4_handle_test(const CommandRequest* cmd, CommandResult* result,
                      int16_t* vCmdL_mV, int16_t* vCmdR_mV);

// Tick @ 1kHz. Needs encoder deltas + gyro.
// velL_mmps/velR_mmps: actual wheel velocity from encoder
// gyroZ_dps: actual yaw rate (°/s)
bool lab4_tick(int16_t* vCmdL_mV, int16_t* vCmdR_mV,
               int16_t velL_mmps, int16_t velR_mmps,
               float gyroZ_dps);

void lab4_stop(void);
bool lab4_is_running(void);

// Runtime gain tuning (from BLE command)
void lab4_set_vel_gains(float Kp, float Kd, float max_mV);
void lab4_set_heading_gains(float Kp, float Ki, float Kd, float max_mV);
void lab4_set_steer_trim(int16_t trim_mV);

#ifdef __cplusplus
}
#endif

#endif // LAB4_PID_H
