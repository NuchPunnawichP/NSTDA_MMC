// ═══════════════════════════════════════════════════════════════════════════════
//  lab1_hw_check.cpp — Lab 1: Hardware Check Implementation
//
//  Motor spin test: CMD sets mV + duration → lab1_tick() counts down → stops.
//  Sensor read tests: instant read → pack into RSP payload → return.
// ═══════════════════════════════════════════════════════════════════════════════
#include "lab1_hw_check.h"
#include "config.h"
#include "hal_motor.h"
#include "hal_encoder.h"
#include "hal_battery.h"
#include "hal_imu.h"
#include "hal_wallsensor.h"
#include "hal_ui.h"
#include "hal_buzzer.h"
#include "sensor_task.h"
#include <string.h>

// ── Helpers: write little-endian into RSP payload ───────────────────────────
static inline void wr_u16(uint8_t* p, uint16_t v) {
    p[0] = (uint8_t)(v);
    p[1] = (uint8_t)(v >> 8);
}
static inline void wr_i16(uint8_t* p, int16_t v) {
    wr_u16(p, (uint16_t)v);
}
static inline void wr_u32(uint8_t* p, uint32_t v) {
    p[0] = (uint8_t)(v);       p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}

// ── Helpers: read little-endian from CMD payload ────────────────────────────
static inline int16_t rd_i16(const uint8_t* p) {
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}
static inline uint16_t rd_u16(const uint8_t* p) {
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

// ═══════════════════════════════════════════════════════════════════════════════
// Motor Spin State (timed test managed by lab1_tick)
// ═══════════════════════════════════════════════════════════════════════════════
static bool     s_motor_running  = false;
static int16_t  s_motor_vCmd_mV  = 0;      // voltage command
static uint8_t  s_motor_select   = 0;      // 0=both, 1=L, 2=R
static uint32_t s_motor_remain   = 0;      // remaining ticks (ms at 1kHz)

// ═══════════════════════════════════════════════════════════════════════════════
// Test Handlers
// ═══════════════════════════════════════════════════════════════════════════════

// ── 0x01: Motor Spin ────────────────────────────────────────────────────────
//  p1 = mV (positive=fwd, negative=rev)
//  p2 = duration_ms
//  p3 = select (0=both, 1=left, 2=right)
static void handle_motor_spin(const uint8_t* payload, CommandResult* res,
                               int16_t* vCmdL, int16_t* vCmdR) {
    int16_t  mV       = rd_i16(&payload[1]);
    uint16_t dur_ms   = rd_u16(&payload[3]);
    uint8_t  select   = payload[5];

    // Validate
    if (dur_ms == 0 || dur_ms > 10000) {
        res->status = RSP_ERR_BAD_ARG;
        return;
    }
    if (mV < -VCMD_MAX_MV || mV > VCMD_MAX_MV) {
        res->status = RSP_ERR_BAD_ARG;
        return;
    }
    if (select > 2) select = 0;

    // Setup timed run
    s_motor_vCmd_mV = mV;
    s_motor_select  = select;
    s_motor_remain  = dur_ms;   // 1 tick = 1ms
    s_motor_running = true;

    // Apply immediately for first tick
    switch (select) {
        case 0: *vCmdL = mV; *vCmdR = mV; break;
        case 1: *vCmdL = mV; *vCmdR = 0;  break;
        case 2: *vCmdL = 0;  *vCmdR = mV; break;
    }

    res->status = RSP_OK;
}

// ── 0x02: LED Set ───────────────────────────────────────────────────────────
//  p1 = R, p2 = G, p3 = B (0–255 each, as int16 → clamp)
static void handle_led_set(const uint8_t* payload, CommandResult* res) {
    int16_t r = rd_i16(&payload[1]);
    int16_t g = rd_i16(&payload[3]);
    int16_t b = rd_i16(&payload[5]);

    // Clamp to 0–255
    if (r < 0) r = 0; if (r > 255) r = 255;
    if (g < 0) g = 0; if (g > 255) g = 255;
    if (b < 0) b = 0; if (b > 255) b = 255;

    hal_ui_led_set((uint8_t)r, (uint8_t)g, (uint8_t)b);
    res->status = RSP_OK;
}

// ── 0x03: Buzzer ────────────────────────────────────────────────────────────
//  p1 = freq_hz, p2 = duration_ms
static void handle_buzzer(const uint8_t* payload, CommandResult* res) {
    uint16_t freq = rd_u16(&payload[1]);
    uint16_t dur  = rd_u16(&payload[3]);

    if (freq > 10000) freq = 10000;
    if (dur > 5000) dur = 5000;

    hal_buzzer_tone(freq, dur);
    res->status = RSP_OK;
}

// ── 0x04: Read Wall Sensors ─────────────────────────────────────────────────
//  RSP payload: wallL(u16), wallFL(u16), wallFR(u16), wallR(u16) = 8 bytes
static void handle_read_sensors(CommandResult* res) {
    uint16_t walls[4];
    sensor_get_wall_all(walls);

    wr_u16(&res->rsp_payload[0], walls[0]);   // Left
    wr_u16(&res->rsp_payload[2], walls[1]);   // Front-Left
    wr_u16(&res->rsp_payload[4], walls[2]);   // Front-Right
    wr_u16(&res->rsp_payload[6], walls[3]);   // Right
    res->rsp_payload[8] = sensor_get_wall_bitmap();
    res->rsp_payload_len = 9;
    res->status = RSP_OK;
}

// ── 0x05: Read Battery ──────────────────────────────────────────────────────
//  RSP payload: vBat_mV(u16), vBat_avg(u16), level(u8) = 5 bytes
static void handle_read_battery(CommandResult* res) {
    uint16_t raw = hal_battery_read_mV();
    uint16_t avg = hal_battery_get_avg_mV();
    uint8_t  lvl = hal_battery_get_level();

    wr_u16(&res->rsp_payload[0], raw);
    wr_u16(&res->rsp_payload[2], avg);
    res->rsp_payload[4] = lvl;
    res->rsp_payload_len = 5;
    res->status = RSP_OK;
}

// ── 0x06: Read IMU ──────────────────────────────────────────────────────────
//  RSP payload: gyroX(i16), gyroY(i16), gyroZ(i16), accelX(i16), accelY(i16), accelZ(i16) = 12 bytes
//  Values are raw register values (not calibrated) for Lab 1 inspection
static void handle_read_imu(CommandResult* res) {
    if (!hal_imu_is_init()) {
        res->status = RSP_ERR_LOCKED;
        return;
    }

    const ImuRawData* raw = hal_imu_get_raw();
    wr_i16(&res->rsp_payload[0],  raw->gyroX);
    wr_i16(&res->rsp_payload[2],  raw->gyroY);
    wr_i16(&res->rsp_payload[4],  raw->gyroZ);
    wr_i16(&res->rsp_payload[6],  raw->accelX);
    wr_i16(&res->rsp_payload[8],  raw->accelY);
    wr_i16(&res->rsp_payload[10], raw->accelZ);
    res->rsp_payload_len = 12;
    res->status = RSP_OK;
}

// ── 0x07: Read Buttons ──────────────────────────────────────────────────────
//  RSP payload: start(u8), mode(u8), dip(u8), encL(i32), encR(i32) = 11 bytes
static void handle_read_buttons(CommandResult* res) {
    res->rsp_payload[0] = hal_ui_btn_start() ? 1 : 0;
    res->rsp_payload[1] = hal_ui_btn_mode() ? 1 : 0;
    res->rsp_payload[2] = hal_ui_dip_read();

    // Bonus: include absolute encoder counts for Lab 1 verification
    int32_t eL = hal_encoder_count_L();
    int32_t eR = hal_encoder_count_R();
    wr_u32(&res->rsp_payload[3], (uint32_t)eL);
    wr_u32(&res->rsp_payload[7], (uint32_t)eR);
    res->rsp_payload_len = 11;
    res->status = RSP_OK;
}

// ═══════════════════════════════════════════════════════════════════════════════
// Public API
// ═══════════════════════════════════════════════════════════════════════════════

bool lab1_handle_test(const CommandRequest* cmd, CommandResult* result,
                      int16_t* vCmdL_mV, int16_t* vCmdR_mV) {
    uint8_t test_id = cmd->payload[0];

    // Initialize result
    memset(result, 0, sizeof(*result));
    result->opcode = cmd->opcode;  // CMD_RUN_TEST
    result->req_id = cmd->req_id;
    result->status = RSP_OK;

    switch (test_id) {
    case TEST_MOTOR_SPIN:
        handle_motor_spin(cmd->payload, result, vCmdL_mV, vCmdR_mV);
        return true;

    case TEST_LED_SET:
        handle_led_set(cmd->payload, result);
        return true;

    case TEST_LED_BRIGHTNESS: {
        // p1 = brightness (0–255)
        int16_t bri = rd_i16(&cmd->payload[1]);
        if (bri < 0) bri = 0;
        if (bri > 255) bri = 255;
        hal_ui_led_brightness((uint8_t)bri);
        // RSP: current brightness
        result->rsp_payload[0] = hal_ui_led_get_brightness();
        result->rsp_payload_len = 1;
        return true;
    }

    case TEST_BUZZER:
        handle_buzzer(cmd->payload, result);
        return true;

    case TEST_READ_SENSORS:
        handle_read_sensors(result);
        return true;

    case TEST_READ_BATTERY:
        handle_read_battery(result);
        return true;

    case TEST_READ_IMU:
        handle_read_imu(result);
        return true;

    case TEST_READ_BUTTONS:
        handle_read_buttons(result);
        return true;

    // ── Calibration Commands (read-only, no ARM required) ────────────────
    case TEST_CAL_GYRO: {
        // Non-blocking gyro calibration via SensorTask
        // First call: starts calibration → RSP with status=0 (started)
        // If already running: RSP_ERR_BUSY
        // If done from previous: returns results + resets
        uint8_t cal_st = sensor_gyro_cal_status();
        uint8_t* rp = result->rsp_payload;

        if (cal_st == 2) {
            // Done — return results and reset
            wr_i16(rp + 0, (int16_t)(hal_imu_get_gyro_offset_z() * 100.0f));
            wr_i16(rp + 2, (int16_t)(hal_imu_get_gyro_offset_x() * 100.0f));
            wr_i16(rp + 4, (int16_t)(hal_imu_get_gyro_offset_y() * 100.0f));
            rp[6] = hal_imu_is_calibrated() ? 1 : 0;
            rp[7] = 2;  // status: done
            result->rsp_payload_len = 8;
        } else if (cal_st == 1) {
            // Busy — tell GUI to wait
            rp[7] = 1;  // status: busy
            result->rsp_payload_len = 8;
        } else {
            // Idle — start calibration
            uint16_t n_samples = 500;
            if (cmd->payload[1] > 0) {
                n_samples = (uint16_t)cmd->payload[1] * 10;
            }
            sensor_request_gyro_cal(n_samples);
            rp[7] = 0;  // status: started
            result->rsp_payload_len = 8;
        }
        return true;
    }
    case TEST_CAL_WALLS: {
        // Non-blocking wall measurement via SensorTask
        uint8_t cal_st = sensor_wall_cal_status();
        uint8_t* rp = result->rsp_payload;

        if (cal_st == 2) {
            // Done — return raw averages + current offsets
            wr_u16(rp + 0, hal_wall_get_cal_raw(SNS_POS_LEFT));
            wr_u16(rp + 2, hal_wall_get_cal_raw(SNS_POS_FRONT_LEFT));
            wr_u16(rp + 4, hal_wall_get_cal_raw(SNS_POS_FRONT_RIGHT));
            wr_u16(rp + 6, hal_wall_get_cal_raw(SNS_POS_RIGHT));
            rp[8]  = (int8_t)constrain(hal_wall_get_offset(SNS_POS_LEFT),       -127, 127);
            rp[9]  = (int8_t)constrain(hal_wall_get_offset(SNS_POS_FRONT_LEFT), -127, 127);
            rp[10] = (int8_t)constrain(hal_wall_get_offset(SNS_POS_FRONT_RIGHT),-127, 127);
            rp[11] = (int8_t)constrain(hal_wall_get_offset(SNS_POS_RIGHT),      -127, 127);
            rp[12] = 2;  // status: done
            result->rsp_payload_len = 13;
        } else if (cal_st == 1) {
            rp[12] = 1;  // status: busy
            result->rsp_payload_len = 13;
        } else {
            uint16_t n_samples = 50;
            if (cmd->payload[1] > 0) {
                n_samples = (uint16_t)cmd->payload[1] * 10;
            }
            sensor_request_wall_cal(n_samples);
            rp[12] = 0;  // status: started
            result->rsp_payload_len = 13;
        }
        return true;
    }
    case TEST_WALL_SET_OFFSET: {
        // Set wall sensor offset for one position
        // payload[1] = position (0=L, 1=FL, 2=FR, 3=R)
        // payload[2] = offset_mm (int8_t, signed: -127 to +127)
        uint8_t pos = cmd->payload[1];
        int8_t  off = (int8_t)cmd->payload[2];
        if (pos < WALL_POS_COUNT) {
            hal_wall_set_offset(pos, (int16_t)off);
        }
        // RSP: echo pos(u8) offset(i8) + all current offsets
        uint8_t* rp = result->rsp_payload;
        rp[0] = pos;
        rp[1] = off;
        rp[2] = (int8_t)constrain(hal_wall_get_offset(SNS_POS_LEFT),       -127, 127);
        rp[3] = (int8_t)constrain(hal_wall_get_offset(SNS_POS_FRONT_LEFT), -127, 127);
        rp[4] = (int8_t)constrain(hal_wall_get_offset(SNS_POS_FRONT_RIGHT),-127, 127);
        rp[5] = (int8_t)constrain(hal_wall_get_offset(SNS_POS_RIGHT),      -127, 127);
        result->rsp_payload_len = 6;
        return true;
    }
    case TEST_CAL_QUERY: {
        // Query current calibration values (no measurement)
        uint8_t* rp = result->rsp_payload;
        wr_i16(rp + 0, (int16_t)(hal_imu_get_gyro_offset_z() * 100.0f));
        rp[2] = hal_imu_is_calibrated() ? 1 : 0;
        // Wall offsets
        rp[3]  = (int8_t)constrain(hal_wall_get_offset(SNS_POS_LEFT),       -127, 127);
        rp[4]  = (int8_t)constrain(hal_wall_get_offset(SNS_POS_FRONT_LEFT), -127, 127);
        rp[5]  = (int8_t)constrain(hal_wall_get_offset(SNS_POS_FRONT_RIGHT),-127, 127);
        rp[6]  = (int8_t)constrain(hal_wall_get_offset(SNS_POS_RIGHT),      -127, 127);
        // Wall raw averages from last calibration
        wr_u16(rp + 7, hal_wall_get_cal_raw(SNS_POS_LEFT));
        wr_u16(rp + 9, hal_wall_get_cal_raw(SNS_POS_FRONT_LEFT));
        // remaining 2 bytes not enough for FR + R, but can fit FL raw
        rp[11] = 0; rp[12] = 0;
        result->rsp_payload_len = 13;
        return true;
    }

    default:
        return false;   // not a Lab 1 test
    }
}

bool lab1_tick(int16_t* vCmdL_mV, int16_t* vCmdR_mV) {
    if (!s_motor_running) {
        *vCmdL_mV = 0;
        *vCmdR_mV = 0;
        return false;
    }

    // Countdown
    if (s_motor_remain > 0) {
        s_motor_remain--;

        // Output current command
        switch (s_motor_select) {
            case 0: *vCmdL_mV = s_motor_vCmd_mV; *vCmdR_mV = s_motor_vCmd_mV; break;
            case 1: *vCmdL_mV = s_motor_vCmd_mV; *vCmdR_mV = 0; break;
            case 2: *vCmdL_mV = 0;               *vCmdR_mV = s_motor_vCmd_mV; break;
        }
        return true;
    } else {
        // Time's up — stop
        s_motor_running = false;
        s_motor_vCmd_mV = 0;
        *vCmdL_mV = 0;
        *vCmdR_mV = 0;
        return false;
    }
}

void lab1_stop(void) {
    s_motor_running = false;
    s_motor_vCmd_mV = 0;
    s_motor_remain  = 0;
}

bool lab1_is_running(void) {
    return s_motor_running;
}
