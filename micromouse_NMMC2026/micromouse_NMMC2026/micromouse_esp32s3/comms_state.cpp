// ═══════════════════════════════════════════════════════════════════════════════
//  comms_state.cpp — Communication State & Queue Owner Implementation
//
//  SINGLE OWNER of all cross-core shared resources.
//  Queues created exactly once. portMUX for snapshot. Atomics for counters.
// ═══════════════════════════════════════════════════════════════════════════════
#include "comms_state.h"
#include "config.h"
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <atomic>

// ─────────────────────────────────────────────────────────────────────────────
// Queues — created once, owned here
// ─────────────────────────────────────────────────────────────────────────────
static QueueHandle_t s_cmdQ = NULL;
static QueueHandle_t s_rspQ = NULL;

// ─────────────────────────────────────────────────────────────────────────────
// SharedSnapshot + portMUX spinlock
// ─────────────────────────────────────────────────────────────────────────────
static SharedSnapshot s_snapshot;
static portMUX_TYPE   s_snapshot_mux = portMUX_INITIALIZER_UNLOCKED;

// ─────────────────────────────────────────────────────────────────────────────
// Atomic cross-core state
// ─────────────────────────────────────────────────────────────────────────────
static std::atomic<bool>     s_ble_connected{false};
static std::atomic<uint16_t> s_mtu{23};
static std::atomic<uint16_t> s_fast_hz{DEFAULT_FAST_HZ};
static std::atomic<uint16_t> s_slow_hz{DEFAULT_SLOW_HZ};
static std::atomic<uint8_t>  s_stream_mask{DEFAULT_STREAM_MASK};
static std::atomic<uint16_t> s_watchdog_ms{DEFAULT_WATCHDOG_MS};

// Diagnostic counters
static std::atomic<uint16_t> s_rx_drop{0};
static std::atomic<uint16_t> s_crc_fail{0};

// ─────────────────────────────────────────────────────────────────────────────
// Init — call ONCE from setup()
// ─────────────────────────────────────────────────────────────────────────────
void comms_state_init(void) {
    // Create queues with exact struct sizes (prevents silent truncation)
    s_cmdQ = xQueueCreate(CMD_QUEUE_DEPTH, sizeof(CommandRequest));
    s_rspQ = xQueueCreate(RSP_QUEUE_DEPTH, sizeof(CommandResult));

    // Zero snapshot
    memset(&s_snapshot, 0, sizeof(s_snapshot));

    // Reset atomics to defaults
    s_ble_connected.store(false);
    s_mtu.store(23);
    s_fast_hz.store(DEFAULT_FAST_HZ);
    s_slow_hz.store(DEFAULT_SLOW_HZ);
    s_stream_mask.store(DEFAULT_STREAM_MASK);
    s_watchdog_ms.store(DEFAULT_WATCHDOG_MS);
    s_rx_drop.store(0);
    s_crc_fail.store(0);
}

// ─────────────────────────────────────────────────────────────────────────────
// Command Mailbox
// ─────────────────────────────────────────────────────────────────────────────
QueueHandle_t comms_get_cmd_queue(void) { return s_cmdQ; }

bool comms_cmd_enqueue(const CommandRequest* cmd) {
    if (!s_cmdQ || !cmd) return false;
    // Non-blocking send (from Core0)
    if (xQueueSend(s_cmdQ, cmd, 0) != pdTRUE) {
        comms_inc_rx_drop();
        return false;
    }
    return true;
}

bool comms_cmd_dequeue(CommandRequest* cmd) {
    if (!s_cmdQ || !cmd) return false;
    // Non-blocking receive (from Core1)
    return (xQueueReceive(s_cmdQ, cmd, 0) == pdTRUE);
}

// ─────────────────────────────────────────────────────────────────────────────
// Response Queue
// ─────────────────────────────────────────────────────────────────────────────
QueueHandle_t comms_get_rsp_queue(void) { return s_rspQ; }

bool comms_rsp_enqueue(const CommandResult* rsp) {
    if (!s_rspQ || !rsp) return false;
    return (xQueueSend(s_rspQ, rsp, 0) == pdTRUE);
}

bool comms_rsp_dequeue(CommandResult* rsp) {
    if (!s_rspQ || !rsp) return false;
    return (xQueueReceive(s_rspQ, rsp, 0) == pdTRUE);
}

// ─────────────────────────────────────────────────────────────────────────────
// SharedSnapshot — portMUX guarded memcpy
//   CRITICAL: lock scope is ONLY the memcpy. Nothing else inside.
// ─────────────────────────────────────────────────────────────────────────────
void comms_snapshot_write(const SharedSnapshot* ss) {
    if (!ss) return;
    portENTER_CRITICAL(&s_snapshot_mux);
    memcpy(&s_snapshot, ss, sizeof(SharedSnapshot));
    portEXIT_CRITICAL(&s_snapshot_mux);
}

void comms_snapshot_read(SharedSnapshot* out) {
    if (!out) return;
    portENTER_CRITICAL(&s_snapshot_mux);
    memcpy(out, &s_snapshot, sizeof(SharedSnapshot));
    portEXIT_CRITICAL(&s_snapshot_mux);
}

// ─────────────────────────────────────────────────────────────────────────────
// Atomic state accessors
// ─────────────────────────────────────────────────────────────────────────────
void     comms_set_ble_connected(bool c)  { s_ble_connected.store(c); }
bool     comms_get_ble_connected(void)    { return s_ble_connected.load(); }

void     comms_set_mtu(uint16_t m)        { s_mtu.store(m); }
uint16_t comms_get_mtu(void)              { return s_mtu.load(); }

void     comms_set_fast_hz(uint16_t hz)   { s_fast_hz.store(hz); }
uint16_t comms_get_fast_hz(void)          { return s_fast_hz.load(); }

void     comms_set_slow_hz(uint16_t hz)   { s_slow_hz.store(hz); }
uint16_t comms_get_slow_hz(void)          { return s_slow_hz.load(); }

void     comms_set_stream_mask(uint8_t m) { s_stream_mask.store(m); }
uint8_t  comms_get_stream_mask(void)      { return s_stream_mask.load(); }

void     comms_set_watchdog_ms(uint16_t m){ s_watchdog_ms.store(m); }
uint16_t comms_get_watchdog_ms(void)      { return s_watchdog_ms.load(); }

// ─────────────────────────────────────────────────────────────────────────────
// Diagnostic counters (atomic increment, safe from any core)
// ─────────────────────────────────────────────────────────────────────────────
void     comms_inc_rx_drop(void)    { s_rx_drop.fetch_add(1); }
uint16_t comms_get_rx_drop(void)    { return s_rx_drop.load(); }

void     comms_inc_crc_fail(void)   { s_crc_fail.fetch_add(1); }
uint16_t comms_get_crc_fail(void)   { return s_crc_fail.load(); }

void comms_reset_counters(void) {
    s_rx_drop.store(0);
    s_crc_fail.store(0);
}
