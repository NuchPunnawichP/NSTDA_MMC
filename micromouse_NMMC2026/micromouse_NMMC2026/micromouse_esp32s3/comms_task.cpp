// ═══════════════════════════════════════════════════════════════════════════════
//  comms_task.cpp — Communications Task Implementation (Core0)
//
//  Main loop (~every 5ms):
//    1. Drain raw CMD queue → bridge_process_cmd() for each
//    2. Drain RSP queue → build RspPacket → notify BLE
//    3. At FAST rate → read snapshot → build FastPacket → notify BLE
//    4. At SLOW rate → read snapshot + counters → build SlowPacket → notify
//    5. Watchdog check → if exceeded, enqueue DISARM to mailbox
//    6. Handle disconnect → re-advertise after delay
// ═══════════════════════════════════════════════════════════════════════════════
#include "comms_task.h"
#include "comms_state.h"
#include "ble_service.h"
#include "telemetry_bridge.h"
#include "config.h"
#include "protocol.h"

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

// ── External: raw CMD queue from ble_service.cpp ────────────────────────────
extern QueueHandle_t ble_get_raw_cmd_queue(void);

// ── Watchdog state ──────────────────────────────────────────────────────────
static uint32_t s_last_valid_cmd_ms = 0;
static bool     s_wd_disarm_sent    = false;

// ── Sequence counters ───────────────────────────────────────────────────────
static uint8_t  s_rsp_seq  = 0;
static uint8_t  s_fast_seq = 0;
static uint8_t  s_slow_seq = 0;

// ── Rate control ────────────────────────────────────────────────────────────
static uint32_t s_last_fast_ms = 0;
static uint32_t s_last_slow_ms = 0;

// ── Disconnect handling ─────────────────────────────────────────────────────
static bool     s_was_connected   = false;
static uint32_t s_disconnect_ms   = 0;
#define READVERTISE_DELAY_MS    500

// ─────────────────────────────────────────────────────────────────────────────
// Helper: send disarm command via mailbox (for watchdog)
// ─────────────────────────────────────────────────────────────────────────────
static void comms_send_disarm(void) {
    CommandRequest cmd;
    memset(&cmd, 0, sizeof(cmd));
    cmd.opcode = CMD_DISARM;
    cmd.req_id = 0xFFFF;  // special req_id for watchdog-generated disarm
    comms_cmd_enqueue(&cmd);
}

// ─────────────────────────────────────────────────────────────────────────────
// CommsTask main function
// ─────────────────────────────────────────────────────────────────────────────
static void commsTaskFunc(void* param) {
    (void)param;

    QueueHandle_t rawQ = ble_get_raw_cmd_queue();
    s_last_valid_cmd_ms = millis();

    // ── Small struct for raw queue items ──
    typedef struct { uint8_t data[PROTO_PACKET_SIZE]; } RawCmdBuf;

    for (;;) {
        uint32_t now = millis();
        bool connected = comms_get_ble_connected();

        // ════════════════════════════════════════════════════════════════════
        // 1. Process incoming raw CMD packets
        // ════════════════════════════════════════════════════════════════════
        if (rawQ != NULL) {
            RawCmdBuf buf;
            // Drain up to 4 per cycle to keep responsive but not hog CPU
            for (int i = 0; i < 4; i++) {
                if (xQueueReceive(rawQ, &buf, 0) != pdTRUE) break;
                if (bridge_process_cmd(buf.data)) {
                    s_last_valid_cmd_ms = now;
                    s_wd_disarm_sent = false;   // reset watchdog flag
                }
            }
        }

        // ════════════════════════════════════════════════════════════════════
        // 2. Send pending RSP packets
        // ════════════════════════════════════════════════════════════════════
        if (connected) {
            CommandResult res;
            // Drain all pending RSPs (typically 0-2)
            while (comms_rsp_dequeue(&res)) {
                RspPacket rsp;
                proto_build_rsp(&rsp, s_rsp_seq++, &res);
                ble_notify_rsp(&rsp);
            }
        }

        // ════════════════════════════════════════════════════════════════════
        // 3. FAST telemetry streaming
        // ════════════════════════════════════════════════════════════════════
        if (connected) {
            uint8_t mask = comms_get_stream_mask();
            uint16_t fast_hz = comms_get_fast_hz();

            if ((mask & STREAM_FAST) && fast_hz > 0) {
                uint32_t period_ms = 1000 / fast_hz;
                if (period_ms < 5) period_ms = 5;  // cap at 200Hz

                if ((now - s_last_fast_ms) >= period_ms) {
                    s_last_fast_ms = now;

                    SharedSnapshot ss;
                    comms_snapshot_read(&ss);

                    FastPacket pkt;
                    proto_build_fast(&pkt, s_fast_seq++, &ss);
                    ble_notify_fast(&pkt);
                    // If notify fails (congestion), we just drop — Iron Rule backpressure
                }
            }
        }

        // ════════════════════════════════════════════════════════════════════
        // 4. SLOW diagnostics streaming
        // ════════════════════════════════════════════════════════════════════
        if (connected) {
            uint8_t mask = comms_get_stream_mask();
            uint16_t slow_hz = comms_get_slow_hz();

            if ((mask & STREAM_SLOW) && slow_hz > 0) {
                uint32_t period_ms = 1000 / slow_hz;
                if (period_ms < 50) period_ms = 50;  // cap at 20Hz

                if ((now - s_last_slow_ms) >= period_ms) {
                    s_last_slow_ms = now;

                    SharedSnapshot ss;
                    comms_snapshot_read(&ss);

                    uint16_t wd_age = (uint16_t)(now - s_last_valid_cmd_ms);

                    SlowPacket pkt;
                    proto_build_slow(&pkt, s_slow_seq++, &ss,
                                     comms_get_rx_drop(),
                                     comms_get_crc_fail(),
                                     wd_age);
                    ble_notify_slow(&pkt);
                }
            }
        }

        // ════════════════════════════════════════════════════════════════════
        // 5. BLE Watchdog
        // ════════════════════════════════════════════════════════════════════
        if (connected) {
            uint16_t wd_ms = comms_get_watchdog_ms();
            if (wd_ms > 0) {
                uint32_t age = now - s_last_valid_cmd_ms;
                if (age > wd_ms && !s_wd_disarm_sent) {
                    comms_send_disarm();
                    s_wd_disarm_sent = true;
                }
            }
        }

        // ════════════════════════════════════════════════════════════════════
        // 6. Disconnect handling: re-advertise
        // ════════════════════════════════════════════════════════════════════
        if (s_was_connected && !connected) {
            // Just disconnected
            s_disconnect_ms = now;
            // Send disarm via mailbox (disconnect safety)
            comms_send_disarm();
            s_wd_disarm_sent = true;
        }
        if (!connected && s_disconnect_ms > 0) {
            if ((now - s_disconnect_ms) > READVERTISE_DELAY_MS) {
                ble_start_advertising();
                s_disconnect_ms = 0;  // only once
            }
        }
        s_was_connected = connected;

        // ════════════════════════════════════════════════════════════════════
        // Sleep — 5ms base loop gives good responsiveness without busy-wait
        // ════════════════════════════════════════════════════════════════════
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Public: start the task
// ─────────────────────────────────────────────────────────────────────────────
void comms_task_start(void) {
    xTaskCreatePinnedToCore(
        commsTaskFunc,
        "CommsTask",
        STACK_COMMS,
        NULL,
        PRIO_COMMS,
        NULL,
        CORE_COMMS
    );
}
