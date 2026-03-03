// ═══════════════════════════════════════════════════════════════════════════════
//  comms_state.h — Communication State & Queue Owner
//
//  Layer: Core Infrastructure
//  Dependencies: protocol.h, FreeRTOS
//
//  ★ This is the SINGLE OWNER of:
//    - g_cmdQ (command mailbox: Core0 enqueues, Core1 dequeues)
//    - g_rspQ (response queue: Core1 enqueues, Core0 dequeues + notifies)
//    - SharedSnapshot (Core1 writes, Core0 reads, protected by portMUX)
//    - Atomic cross-core state variables
//
//  Iron Rules:
//    - Queues created ONCE in comms_state_init(). No extern + re-create.
//    - SharedSnapshot protected by portMUX spinlock (gated copy, no torn read)
//    - Lock must NOT wrap notify/queue/blocking calls
//    - BLE callback: copy→enqueue→return. No parsing. No blocking.
// ═══════════════════════════════════════════════════════════════════════════════
#ifndef COMMS_STATE_H
#define COMMS_STATE_H

#include <stdint.h>
#include <stdbool.h>
#include "protocol.h"

// FreeRTOS types (forward — actual include in .cpp)
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#ifdef __cplusplus
extern "C" {
#endif

// ─────────────────────────────────────────────────────────────────────────────
// Initialization — call ONCE from setup(), before tasks start
// ─────────────────────────────────────────────────────────────────────────────
void comms_state_init(void);

// ─────────────────────────────────────────────────────────────────────────────
// Command Mailbox (Core0 → Core1)
//   BLE callback → telemetry_bridge parses → enqueues CommandRequest here
//   ControlTask dequeues at tick boundary
// ─────────────────────────────────────────────────────────────────────────────
QueueHandle_t comms_get_cmd_queue(void);

// Convenience: enqueue a command (returns true if queued, false if full)
// Called from Core0 (telemetry bridge). Non-blocking.
bool comms_cmd_enqueue(const CommandRequest* cmd);

// Convenience: dequeue a command (returns true if got one)
// Called from Core1 (control task) at tick boundary. Non-blocking.
bool comms_cmd_dequeue(CommandRequest* cmd);

// ─────────────────────────────────────────────────────────────────────────────
// Response Queue (Core1 → Core0)
//   ControlTask fills CommandResult after applying CMD → enqueues here
//   CommsTask dequeues → builds RspPacket → notifies BLE
// ─────────────────────────────────────────────────────────────────────────────
QueueHandle_t comms_get_rsp_queue(void);

bool comms_rsp_enqueue(const CommandResult* rsp);
bool comms_rsp_dequeue(CommandResult* rsp);

// ─────────────────────────────────────────────────────────────────────────────
// SharedSnapshot — Core1 writes, Core0 reads
//   Protected by portMUX spinlock. Lock scope must be MINIMAL.
//   Do NOT hold lock while calling notify() or any blocking API.
// ─────────────────────────────────────────────────────────────────────────────

// Core1 calls this to publish latest snapshot (inside step1kHz)
void comms_snapshot_write(const SharedSnapshot* ss);

// Core0 calls this to get a consistent copy (for building FAST/SLOW packets)
void comms_snapshot_read(SharedSnapshot* out);

// ─────────────────────────────────────────────────────────────────────────────
// Atomic Cross-Core State Variables
//   Safe to read/write from any core without lock.
// ─────────────────────────────────────────────────────────────────────────────

// BLE connection state (set by BLE callbacks on Core0)
void comms_set_ble_connected(bool connected);
bool comms_get_ble_connected(void);

// Negotiated MTU size
void comms_set_mtu(uint16_t mtu);
uint16_t comms_get_mtu(void);

// Telemetry rate settings (set by CMD handler, read by CommsTask)
void comms_set_fast_hz(uint16_t hz);
uint16_t comms_get_fast_hz(void);

void comms_set_slow_hz(uint16_t hz);
uint16_t comms_get_slow_hz(void);

// Stream mask (bit0=FAST, bit1=SLOW)
void comms_set_stream_mask(uint8_t mask);
uint8_t comms_get_stream_mask(void);

// Watchdog timeout (ms, 0=disabled)
void comms_set_watchdog_ms(uint16_t ms);
uint16_t comms_get_watchdog_ms(void);

// ─────────────────────────────────────────────────────────────────────────────
// Diagnostic Counters (atomic, increment from anywhere)
// ─────────────────────────────────────────────────────────────────────────────
void     comms_inc_rx_drop(void);       // queue full or bad len
uint16_t comms_get_rx_drop(void);

void     comms_inc_crc_fail(void);
uint16_t comms_get_crc_fail(void);

void     comms_reset_counters(void);

#ifdef __cplusplus
}
#endif

#endif // COMMS_STATE_H
