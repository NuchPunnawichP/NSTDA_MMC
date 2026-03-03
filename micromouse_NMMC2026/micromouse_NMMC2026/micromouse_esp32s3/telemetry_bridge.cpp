// ═══════════════════════════════════════════════════════════════════════════════
//  telemetry_bridge.cpp — Command Parser / Validator Implementation
//
//  Validation pipeline:  len → CRC8 → type → opcode → route
//
//  Routing:
//    "Core0-local" commands (telemetry settings) → apply atomics + enqueue RSP
//    "Core1" commands (motor/test/tune) → enqueue to g_cmdQ
//    Both paths generate a CommandResult that CommsTask sends as RSP.
// ═══════════════════════════════════════════════════════════════════════════════
#include "telemetry_bridge.h"
#include "comms_state.h"
#include "config.h"
#include <string.h>

// ── Helper: read little-endian uint16 from payload ──────────────────────────
static inline uint16_t read_u16(const uint8_t* p) {
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

// ── Helper: read little-endian int16 from payload ───────────────────────────
static inline int16_t read_i16(const uint8_t* p) {
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

// ── Helper: make a quick OK/ERR result and enqueue to RSP queue ─────────────
static void bridge_quick_rsp(uint8_t opcode, uint16_t req_id, uint8_t status) {
    CommandResult res;
    memset(&res, 0, sizeof(res));
    res.opcode  = opcode;
    res.req_id  = req_id;
    res.status  = status;
    res.rsp_payload_len = 0;
    comms_rsp_enqueue(&res);
}

// ═══════════════════════════════════════════════════════════════════════════════
// Public API
// ═══════════════════════════════════════════════════════════════════════════════

bool bridge_process_cmd(const uint8_t raw[20]) {
    // ── Step 1: Validate type field ──
    if (raw[0] != PKT_CMD) {
        comms_inc_rx_drop();
        return false;
    }

    // ── Step 2: Validate CRC8 ──
    uint8_t crc = proto_crc8(raw, 19);
    if (crc != raw[19]) {
        comms_inc_crc_fail();
        return false;
    }

    // ── Step 3: Parse CMD header ──
    const CmdPacket* cmd = (const CmdPacket*)raw;
    uint8_t  opcode = cmd->opcode;
    uint16_t req_id = cmd->req_id;
    const uint8_t* payload = cmd->payload;  // 14 bytes [5..18]

    // ── Step 4: Route based on opcode ──

    switch (opcode) {

    // ─── Core0-local: Telemetry / Link settings ─────────────────────────
    // These only touch atomic state — safe to handle immediately on Core0.
    // Still produce RSP so Python gets confirmation.

    case CMD_SET_FAST_HZ: {
        uint16_t hz = read_u16(&payload[0]);
        if (hz < 1 || hz > 200) {
            bridge_quick_rsp(opcode, req_id, RSP_ERR_BAD_ARG);
        } else {
            comms_set_fast_hz(hz);
            bridge_quick_rsp(opcode, req_id, RSP_OK);
        }
        return true;
    }

    case CMD_SET_SLOW_HZ: {
        uint16_t hz = read_u16(&payload[0]);
        if (hz < 1 || hz > 20) {
            bridge_quick_rsp(opcode, req_id, RSP_ERR_BAD_ARG);
        } else {
            comms_set_slow_hz(hz);
            bridge_quick_rsp(opcode, req_id, RSP_OK);
        }
        return true;
    }

    case CMD_SET_STREAM_MASK: {
        uint8_t mask = payload[0];
        comms_set_stream_mask(mask & 0x03);  // only bit0, bit1 valid
        bridge_quick_rsp(opcode, req_id, RSP_OK);
        return true;
    }

    case CMD_SET_WATCHDOG_MS: {
        uint16_t ms = read_u16(&payload[0]);
        comms_set_watchdog_ms(ms);
        bridge_quick_rsp(opcode, req_id, RSP_OK);
        return true;
    }

    // ─── Core1-bound: everything else → mailbox ─────────────────────────
    // PING, ARM, DISARM, SET_LAB, RUN_TEST, STOP, ABORT, RESET_COUNTERS,
    // SET_FF_*, SET_PD_*, and any unknown opcode
    default: {
        CommandRequest creq;
        creq.opcode = opcode;
        creq.req_id = req_id;
        memcpy(creq.payload, payload, 14);

        if (!comms_cmd_enqueue(&creq)) {
            // Queue full — send BUSY RSP immediately
            bridge_quick_rsp(opcode, req_id, RSP_ERR_BUSY);
        }
        // Note: RSP for these will come from Core1 after processing
        return true;
    }

    } // switch
}
