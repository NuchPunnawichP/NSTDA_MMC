// ═══════════════════════════════════════════════════════════════════════════════
//  telemetry_bridge.h — Command Parser / Validator
//
//  Layer: Core Infrastructure
//  Dependencies: protocol.h, comms_state.h
//
//  ★ Iron Rules:
//    - Parse + validate + enqueue ONLY. Never send RSP. Never notify.
//    - Validation order: len==20 → CRC8 → opcode known → payload valid
//    - If CRC fails → drop + increment counter (no RSP)
//    - If opcode unknown → enqueue anyway (ControlTask returns ERR_UNKNOWN_OP)
//    - Some opcodes (telemetry settings) are handled locally on Core0
//      because they don't affect motor state — but still go through
//      the same validation pipeline.
// ═══════════════════════════════════════════════════════════════════════════════
#ifndef TELEMETRY_BRIDGE_H
#define TELEMETRY_BRIDGE_H

#include <stdint.h>
#include <stdbool.h>
#include "protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

// ─────────────────────────────────────────────────────────────────────────────
// Process one raw 20-byte CMD packet from BLE
//
//  Called by CommsTask after dequeuing from raw CMD queue.
//  Validates, then either:
//    a) Handles locally (telemetry settings) + enqueues RSP directly
//    b) Enqueues CommandRequest to g_cmdQ for Core1 processing
//
//  Returns true if packet was valid (regardless of enqueue success).
//  Returns false if validation failed (CRC/len).
// ─────────────────────────────────────────────────────────────────────────────
bool bridge_process_cmd(const uint8_t raw[20]);

#ifdef __cplusplus
}
#endif

#endif // TELEMETRY_BRIDGE_H
