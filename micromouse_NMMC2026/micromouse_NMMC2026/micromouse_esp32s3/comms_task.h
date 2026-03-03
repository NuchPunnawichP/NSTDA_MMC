// ═══════════════════════════════════════════════════════════════════════════════
//  comms_task.h — Communications Task (Core0)
//
//  Layer: Core Infrastructure
//  Dependencies: comms_state.h, ble_service.h, telemetry_bridge.h
//
//  Responsibilities:
//    1. Dequeue raw CMD packets from BLE → call telemetry_bridge
//    2. Dequeue CommandResult from Core1 → build RspPacket → notify BLE
//    3. Periodically build FAST/SLOW packets from SharedSnapshot → notify BLE
//    4. BLE watchdog: if no valid CMD for X ms → trigger disarm
//    5. Handle BLE disconnect → re-advertise
//
//  ★ Iron Rules:
//    - This is the SINGLE RSP/NOTIFY authority
//    - Runs on Core0 at PRIO_COMMS
//    - Never touches motor state / PID (that's Core1 only)
// ═══════════════════════════════════════════════════════════════════════════════
#ifndef COMMS_TASK_H
#define COMMS_TASK_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ─────────────────────────────────────────────────────────────────────────────
// Start CommsTask — call from setup() after comms_state_init + ble_init
// Creates the FreeRTOS task pinned to Core0.
// ─────────────────────────────────────────────────────────────────────────────
void comms_task_start(void);

#ifdef __cplusplus
}
#endif

#endif // COMMS_TASK_H
