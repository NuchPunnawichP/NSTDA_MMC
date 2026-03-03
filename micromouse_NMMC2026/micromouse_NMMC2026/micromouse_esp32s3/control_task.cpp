// ═══════════════════════════════════════════════════════════════════════════════
//  control_task.cpp — Real-Time Control Task Implementation (Core1, 1kHz)
//
//  Timing architecture:
//    esp_timer (ESP_TIMER_TASK dispatch) fires every 1000µs
//    → callback calls xTaskNotifyGive(controlTaskHandle)
//    → ControlTask wakes from ulTaskNotifyTake()
//    → executes step1kHz()
//    → writes SharedSnapshot under portMUX
//    → goes back to sleep
//
//  ★ FORBIDDEN (causes crash/reboot):
//    - vTaskNotifyGiveFromISR() — esp_timer runs in task context, not ISR
//    - portYIELD_FROM_ISR()     — same reason
//    - vTaskDelayUntil()        — creates jitter, not synchronized to timer
//    - I2C / SPI / slow calls   — delegate to SensorTask
// ═══════════════════════════════════════════════════════════════════════════════
#include "control_task.h"
#include "comms_state.h"
#include "config.h"
#include "protocol.h"

// HAL includes
#include "hal_motor.h"
#include "hal_encoder.h"
#include "hal_battery.h"
#include "hal_imu.h"
#include "hal_wallsensor.h"

// Lab handlers
#include "lab1_hw_check.h"
#include "lab2a_enc_cal.h"
#include "lab2b_motor_char.h"
#include "lab3_motion.h"
#include "lab4_pid.h"
#include "lab5_search.h"
#include "lab6_speedrun.h"

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_timer.h>

// ── Task handle (needed for xTaskNotifyGive from timer callback) ────────────
static TaskHandle_t s_controlTaskHandle = NULL;

// ── esp_timer handle ────────────────────────────────────────────────────────
static esp_timer_handle_t s_timer = NULL;

// ── Control state (single writer: this task only) ───────────────────────────
static bool     s_armed       = false;
static bool     s_test_active = false;
static uint8_t  s_state       = STATE_IDLE;
static uint16_t s_missed_ticks = 0;
static uint16_t s_step_max_us  = 0;

// Motor commands (mV)
static int16_t  s_vCmdL_mV = 0;
static int16_t  s_vCmdR_mV = 0;

// Global: motion profile target velocity (set by active lab tick)
int16_t g_v_target_mmps = 0;

// ── Cached sensor data (updated by step or from SensorTask) ─────────────────
static int16_t  s_velL_mmps = 0;
static int16_t  s_velR_mmps = 0;
static uint16_t s_vBat_mV   = 7400;    // safe default

// ── Velocity estimation: sliding window for smooth readings ─────────────────
// Raw 1ms delta is too quantized (0 or ±130mm/s steps).
// Accumulate over VEL_EST_WINDOW_MS ticks for N× better resolution.
#ifndef VEL_EST_WINDOW_MS
  #define VEL_EST_WINDOW_MS 5
#endif
#define VEL_WIN VEL_EST_WINDOW_MS

static int16_t s_encBufL[VEL_WIN];
static int16_t s_encBufR[VEL_WIN];
static uint8_t s_encBufIdx = 0;
static bool    s_encBufFull = false;

// Last applied CMD info (for SLOW packet)
static uint8_t  s_last_cmd_op     = 0;
static uint8_t  s_last_cmd_status = 0;

// ═══════════════════════════════════════════════════════════════════════════════
// esp_timer callback — runs in TASK context (ESP_TIMER_TASK)
//   ★ Uses xTaskNotifyGive() — NOT FromISR variant
// ═══════════════════════════════════════════════════════════════════════════════
static void IRAM_ATTR timer_callback(void* arg) {
    (void)arg;
    if (s_controlTaskHandle != NULL) {
        // esp_timer with ESP_TIMER_TASK dispatch runs in task context
        // → use xTaskNotifyGive(), NOT vTaskNotifyGiveFromISR()
        xTaskNotifyGive(s_controlTaskHandle);
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
// CMD Processing — dequeue and apply at tick boundary
//   Returns CommandResult to be enqueued back for RSP
// ═══════════════════════════════════════════════════════════════════════════════

// Helper: read little-endian int16 from payload
static inline int16_t rd_i16(const uint8_t* p) {
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}
static inline uint16_t rd_u16(const uint8_t* p) {
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}
// Helper: write little-endian uint32 into payload
static inline void wr_u32(uint8_t* p, uint32_t v) {
    p[0] = (uint8_t)(v);
    p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16);
    p[3] = (uint8_t)(v >> 24);
}

static void apply_command(const CommandRequest* cmd) {
    CommandResult res;
    memset(&res, 0, sizeof(res));
    res.opcode = cmd->opcode;
    res.req_id = cmd->req_id;
    res.status = RSP_OK;
    res.rsp_payload_len = 0;

    switch (cmd->opcode) {

    // ── Session / Safety ────────────────────────────────────────────────
    case CMD_PING: {
        uint32_t up = millis();
        wr_u32(res.rsp_payload, up);
        res.rsp_payload_len = 4;
        break;
    }

    case CMD_ARM:
        if (hal_battery_is_critical()) {
            res.status = RSP_ERR_LOCKED;    // low battery → refuse arm
        } else {
            s_armed = true;
            s_state = STATE_ARMED;
        }
        break;

    case CMD_DISARM:
        lab1_stop();
        lab2b_stop();
        lab3_stop();
        lab4_stop();
        lab5_stop();
        lab6_stop();
        s_armed = false;
        s_test_active = false;
        s_vCmdL_mV = 0;
        s_vCmdR_mV = 0;
        hal_motor_brake();
        s_state = STATE_IDLE;
        break;

    case CMD_SET_LAB:
        // payload[0] = lab_id — store for lab dispatching (future)
        // For now, just acknowledge
        break;

    // ── Test Execution ──────────────────────────────────────────────────
    case CMD_RUN_TEST: {
        // Dispatch to lab handler based on test_id range
        uint8_t test_id = cmd->payload[0];
        CommandResult lab_res;
        bool handled = false;

        // Read-only tests don't require ARM
        bool is_read_only = (test_id >= 0x04 && test_id <= 0x07)   // Lab1 sensors
                         || (test_id >= 0x09 && test_id <= 0x0C)   // Lab1 calibration + offset
                         || (test_id >= 0x10 && test_id <= 0x11)   // Lab2a encoder read
                         || (test_id >= 0x54 && test_id <= 0x56)   // Lab5 read walls/query/maze
                         || (test_id == 0x62);                     // Lab6 speed run query

        if (!is_read_only) {
            // Motor/actuator tests require ARM
            if (!s_armed) {
                res.status = RSP_ERR_DISARMED;
                break;
            }
            if (s_test_active) {
                res.status = RSP_ERR_BUSY;
                break;
            }
        }

        if (test_id >= 0x01 && test_id <= 0x0F) {
            // Lab 1 tests (0x01-0x0F)
            handled = lab1_handle_test(cmd, &lab_res,
                                        &s_vCmdL_mV, &s_vCmdR_mV);
        } else if (test_id >= 0x10 && test_id <= 0x11) {
            // Lab 2a: Encoder calibration (read-only, no ARM)
            handled = lab2a_handle_test(cmd, &lab_res);
        } else if (test_id >= 0x20 && test_id <= 0x22) {
            // Lab 2b: Motor characterization
            handled = lab2b_handle_test(cmd, &lab_res,
                                         &s_vCmdL_mV, &s_vCmdR_mV);
        } else if (test_id >= 0x30 && test_id <= 0x3F) {
            // Lab 3: Motion profiles (open-loop FF)
            handled = lab3_handle_test(cmd, &lab_res,
                                        &s_vCmdL_mV, &s_vCmdR_mV);
        } else if (test_id >= 0x40 && test_id <= 0x4F) {
            // Lab 4: PID-controlled motion
            handled = lab4_handle_test(cmd, &lab_res,
                                        &s_vCmdL_mV, &s_vCmdR_mV);
        } else if (test_id >= 0x50 && test_id <= 0x5F) {
            // Lab 5: Maze search
            handled = lab5_handle_test(cmd, &lab_res,
                                        &s_vCmdL_mV, &s_vCmdR_mV);
        } else if (test_id >= 0x60 && test_id <= 0x6F) {
            // Lab 6: Speed run
            handled = lab6_handle_test(cmd, &lab_res,
                                        &s_vCmdL_mV, &s_vCmdR_mV);
        }

        if (handled) {
            // Use lab's result (status + payload)
            res = lab_res;
            // If motor test started, mark as running
            if (lab1_is_running() || lab2b_is_running() || lab3_is_running() || lab4_is_running() || lab5_is_running() || lab6_is_running()) {
                s_test_active = true;
                s_state = STATE_RUNNING;
            }
        } else {
            res.status = RSP_ERR_UNKNOWN_OP;
        }
        break;
    }

    case CMD_STOP:
        lab1_stop();
        lab2b_stop();
        lab3_stop();
        lab4_stop();
        lab5_stop();
        lab6_stop();
        s_test_active = false;
        s_vCmdL_mV = 0;
        s_vCmdR_mV = 0;
        hal_motor_coast();
        s_state = s_armed ? STATE_ARMED : STATE_IDLE;
        break;

    case CMD_ABORT:
        lab1_stop();
        lab2b_stop();
        lab3_stop();
        lab4_stop();
        lab5_stop();
        lab6_stop();
        s_armed = false;
        s_test_active = false;
        s_vCmdL_mV = 0;
        s_vCmdR_mV = 0;
        hal_motor_brake();
        s_state = STATE_IDLE;
        break;

    case CMD_RESET_COUNTERS:
        hal_encoder_reset();
        comms_reset_counters();
        s_missed_ticks = 0;
        s_step_max_us = 0;
        break;

    // ── Runtime Tuning ──────────────────────────────────────────────────
    case CMD_SET_FF_LINEAR:
    case CMD_SET_FF_ROTATION: {
        // TODO: runtime FF tuning
        break;
    }

    case CMD_SET_PD_LINEAR: {
        // payload: [Kp×100, Kd×100, maxOut_mV, rsvd, rsvd] (i16 LE)
        float Kp = rd_i16(&cmd->payload[0]) / 100.0f;
        float Kd = rd_i16(&cmd->payload[2]) / 100.0f;
        float mx = (float)rd_i16(&cmd->payload[4]);
        lab4_set_vel_gains(Kp, Kd, mx);
        break;
    }
    case CMD_SET_PD_ROTATION:
    case CMD_SET_PD_STEERING: {
        // payload: [Kp×100, Ki×100, Kd×100, maxOut_mV, trim_mV] (i16 LE)
        float Kp = rd_i16(&cmd->payload[0]) / 100.0f;
        float Ki = rd_i16(&cmd->payload[2]) / 100.0f;
        float Kd = rd_i16(&cmd->payload[4]) / 100.0f;
        float mx = (float)rd_i16(&cmd->payload[6]);
        int16_t trim = rd_i16(&cmd->payload[8]);
        lab4_set_heading_gains(Kp, Ki, Kd, mx);
        lab4_set_steer_trim(trim);
        break;
    }
    case CMD_SET_PD_WALL: {
        // TODO: wall PD tuning for Lab 5
        break;
    }

    default:
        res.status = RSP_ERR_UNKNOWN_OP;
        break;
    }

    // Record for diagnostics
    s_last_cmd_op     = cmd->opcode;
    s_last_cmd_status = res.status;

    // Enqueue RSP for CommsTask to send
    comms_rsp_enqueue(&res);
}

// ═══════════════════════════════════════════════════════════════════════════════
// step1kHz — the real-time control step
//   Called every 1ms from the main control loop.
//   ★ Must be fast (<500µs). No I2C. No blocking.
// ═══════════════════════════════════════════════════════════════════════════════
static void step1kHz(void) {
    // ── Read encoders (PCNT hardware — fast, no I2C) ──
    int16_t dL = hal_encoder_delta_L();
    int16_t dR = hal_encoder_delta_R();

    // ── Velocity estimation (mm/s) — sliding window ──
    // Store delta in ring buffer
    s_encBufL[s_encBufIdx] = dL;
    s_encBufR[s_encBufIdx] = dR;
    s_encBufIdx++;
    if (s_encBufIdx >= VEL_WIN) { s_encBufIdx = 0; s_encBufFull = true; }

    // Sum over window
    uint8_t n = s_encBufFull ? VEL_WIN : s_encBufIdx;
    int32_t sumL = 0, sumR = 0;
    for (uint8_t i = 0; i < n; i++) { sumL += s_encBufL[i]; sumR += s_encBufR[i]; }

    // vel = sum_counts × MM_PER_COUNT × (1000 / window_ms)
    float scale = MM_PER_COUNT * (1000.0f / (float)n);
    s_velL_mmps = (int16_t)(sumL * scale);
    s_velR_mmps = (int16_t)(sumR * scale);

    // ── Read gyro Z (for heading PID) ──
    float gyroZ_dps = hal_imu_gyro_z_dps();

    // ── Read battery (fast ADC, no I2C) ──
    // Only update every ~100 ticks (10Hz) to reduce ADC load
    static uint16_t batt_div = 0;
    if (++batt_div >= 100) {
        batt_div = 0;
        hal_battery_update();
    }
    s_vBat_mV = hal_battery_get_avg_mV();

    // ── Safety: check battery ──
    if (hal_battery_is_critical() && s_armed) {
        s_armed = false;
        s_test_active = false;
        s_vCmdL_mV = 0;
        s_vCmdR_mV = 0;
        hal_motor_brake();
        s_state = STATE_ERROR;
    }

    // ── Motor output ──
    if (s_armed) {
        if (s_test_active) {
            // Let lab handlers manage motor commands (chain: lab1→2b→3→4)
            int16_t labL = 0, labR = 0;
            g_v_target_mmps = 0;  // reset — active lab will set it
            bool still_running = lab1_tick(&labL, &labR);
            if (!still_running) {
                still_running = lab2b_tick(&labL, &labR);
            }
            if (!still_running) {
                still_running = lab3_tick(&labL, &labR, dL, dR);
            }
            if (!still_running) {
                still_running = lab4_tick(&labL, &labR,
                                          s_velL_mmps, s_velR_mmps, gyroZ_dps);
            }
            if (!still_running) {
                still_running = lab5_tick(&labL, &labR,
                                          s_velL_mmps, s_velR_mmps, gyroZ_dps);
            }
            if (!still_running) {
                still_running = lab6_tick(&labL, &labR,
                                          s_velL_mmps, s_velR_mmps, gyroZ_dps);
            }
            s_vCmdL_mV = labL;
            s_vCmdR_mV = labR;

            if (!still_running) {
                // Test finished on its own (timed out)
                s_test_active = false;
                s_state = STATE_ARMED;
            }
        }
        // Apply motor command (whether from test or zero)
        hal_motor_set_mV(s_vCmdL_mV, s_vCmdR_mV, s_vBat_mV);
    } else {
        // Ensure motors are off when disarmed
        if (s_vCmdL_mV != 0 || s_vCmdR_mV != 0) {
            s_vCmdL_mV = 0;
            s_vCmdR_mV = 0;
            hal_motor_coast();
        }
    }

    // ── Build state flags ──
    uint8_t state = s_state & 0x0F;
    if (s_armed)       state |= FLAG_ARMED;
    if (s_test_active) state |= FLAG_RUNNING;
    if (hal_battery_is_critical()) state |= FLAG_LOW_BATT;

    // ── Build active feature flags ──
    uint8_t feat = 0;
    if (hal_motor_is_init())     feat |= FEAT_MOTOR_OK;
    if (hal_encoder_is_init())   feat |= FEAT_ENCODER_OK;
    if (hal_imu_is_init())       feat |= FEAT_IMU_OK;
    if (hal_wall_is_init())      feat |= FEAT_WALL_SNS_OK;
    if (comms_get_ble_connected()) feat |= FEAT_BLE_CONN;
    if (s_armed)                 feat |= FEAT_ARMED;
    if (s_test_active)           feat |= FEAT_TEST_ACTIVE;

    // ── Write SharedSnapshot (portMUX inside comms_snapshot_write) ──
    SharedSnapshot ss;
    ss.t_ms          = (uint16_t)(millis() & 0xFFFF);
    ss.vCmdL_mV      = s_vCmdL_mV;
    ss.vCmdR_mV      = s_vCmdR_mV;
    ss.velL_mmps     = s_velL_mmps;
    ss.velR_mmps     = s_velR_mmps;
    ss.v_target_mmps = g_v_target_mmps;
    ss.gyroZ_dps10   = hal_imu_gyro_z_dps10();
    ss.encL_delta    = (int8_t)CONSTRAIN(dL, -127, 127);
    ss.encR_delta    = (int8_t)CONSTRAIN(dR, -127, 127);
    ss.state         = state;
    ss.missed_ticks  = s_missed_ticks;
    ss.step_max_us   = s_step_max_us;
    ss.vBat_avg_mV   = hal_battery_get_avg_mV();
    ss.last_cmd_op   = s_last_cmd_op;
    ss.last_cmd_status = s_last_cmd_status;
    ss.active_flags  = feat;

    comms_snapshot_write(&ss);
}

// ═══════════════════════════════════════════════════════════════════════════════
// ControlTask main function — the heartbeat of the robot
// ═══════════════════════════════════════════════════════════════════════════════
static void controlTaskFunc(void* param) {
    (void)param;

    for (;;) {
        // ── Block until timer notification ──
        // ulTaskNotifyTake blocks efficiently until xTaskNotifyGive from timer
        uint32_t count = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(5));
        // pdMS_TO_TICKS(5) is a safety timeout — if timer stops, we don't hang forever

        if (count == 0) {
            // Timeout — timer didn't fire. Count as missed tick.
            s_missed_ticks++;
            continue;
        }

        // Handle multiple notifications (catch-up if we missed some)
        if (count > 1) {
            s_missed_ticks += (count - 1);
        }

        uint32_t t0 = micros();

        // ── Dequeue and apply any pending commands (tick boundary) ──
        CommandRequest cmd;
        while (comms_cmd_dequeue(&cmd)) {
            apply_command(&cmd);
        }

        // ── Execute 1kHz control step ──
        step1kHz();

        // ── Track execution time ──
        uint32_t dt_us = micros() - t0;
        if (dt_us > s_step_max_us) {
            s_step_max_us = (uint16_t)(dt_us > 65535 ? 65535 : dt_us);
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
// Public: start the control task + timer
// ═══════════════════════════════════════════════════════════════════════════════
void control_task_start(void) {
    // ── Create the FreeRTOS task first (so handle is ready for timer) ──
    xTaskCreatePinnedToCore(
        controlTaskFunc,
        "ControlTask",
        STACK_CONTROL,
        NULL,
        PRIO_CONTROL,
        &s_controlTaskHandle,
        CORE_CONTROL
    );

    // ── Create esp_timer ──
    // dispatch_method = ESP_TIMER_TASK → callback runs in esp_timer task context
    // → safe to use xTaskNotifyGive() (NOT FromISR!)
    esp_timer_create_args_t timer_args = {};
    timer_args.callback        = timer_callback;
    timer_args.arg             = NULL;
    timer_args.dispatch_method = ESP_TIMER_TASK;
    timer_args.name            = "ctrl_1kHz";

    esp_timer_create(&timer_args, &s_timer);

    // Start periodic timer: 1000µs = 1ms = 1kHz
    esp_timer_start_periodic(s_timer, CONTROL_PERIOD_US);
}
