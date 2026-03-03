#pragma once
// ═══════════════════════════════════════════════════════════════════════════════
//  protocol.h — Micromouse BLE Protocol v2.0
//  "Single source of truth" for FW ↔ Python communication
//
//  Target: ESP32-S3, Arduino core 3.3.5, BLEDevice.h (NOT NimBLE)
//  Design: ALL packets = fixed 20 bytes, CRC8/ATM @ byte[19]
//
//  ★ Iron Rules ★
//    - Wire integers only (int16 scaled). No float on wire.
//    - duty = vCmd_mV / vBat_mV (never raw PWM in upper layers)
//    - RX accepts len==20 only. Everything else → drop + counter.
//    - Every notify char needs CCCD (BLE2902) for iOS.
//    - CMD → Mailbox → Core1 apply at tick boundary.
//    - RSP sent by CommsTask (Core0) ONLY. Single RSP authority.
// ═══════════════════════════════════════════════════════════════════════════════

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// ─────────────────────────────────────────────────────────────────────────────
// Protocol Constants
// ─────────────────────────────────────────────────────────────────────────────
#define PROTO_VER               2       // Protocol version (v2.0)
#define PROTO_PACKET_SIZE       20      // ALL packets = 20 bytes
#define PROTO_PAYLOAD_CMD       14      // CMD payload bytes [5..18]
#define PROTO_PAYLOAD_RSP       13      // RSP payload bytes [6..18]
#define PROTO_PAYLOAD_FAST      18      // FAST data bytes [2..19] minus crc
#define PROTO_PAYLOAD_SLOW      18      // SLOW data bytes [2..19] minus crc

// Firmware version (student project)
#define FW_VER_MAJOR            2
#define FW_VER_MINOR            0
#define FW_VER                  ((FW_VER_MAJOR << 8) | FW_VER_MINOR)

// Scaling contract ID — bump when scaling table changes
#define SCALE_ID                1

// ─────────────────────────────────────────────────────────────────────────────
// Packet Type IDs (byte[0])
// ─────────────────────────────────────────────────────────────────────────────
#define PKT_FAST                0x01
#define PKT_SLOW                0x02
#define PKT_CAPS                0x10
#define PKT_CMD                 0x20
#define PKT_RSP                 0x30

// ─────────────────────────────────────────────────────────────────────────────
// GATT UUIDs — 128-bit, custom base
//   Base: MM00xxxx-5A19-40B6-926C-F22D00D00000
// ─────────────────────────────────────────────────────────────────────────────
#define UUID_SERVICE            "A0000000-5A19-40B6-926C-F22D00D00000"
#define UUID_CAPS               "A0000001-5A19-40B6-926C-F22D00D00000"
#define UUID_CMD                "A0000002-5A19-40B6-926C-F22D00D00000"
#define UUID_RSP                "A0000003-5A19-40B6-926C-F22D00D00000"
#define UUID_FAST               "A0000004-5A19-40B6-926C-F22D00D00000"
#define UUID_SLOW               "A0000005-5A19-40B6-926C-F22D00D00000"
#define UUID_CFG                "A0000006-5A19-40B6-926C-F22D00D00000"

// ─────────────────────────────────────────────────────────────────────────────
// Command Opcodes (CMD byte[2])
// ─────────────────────────────────────────────────────────────────────────────

// --- Session / Safety (0x01–0x0F) ---
#define CMD_PING                0x01    // → RSP payload: u32 uptime_ms
#define CMD_ARM                 0x02    // → enable motor output
#define CMD_DISARM              0x03    // → hard stop + disable motors
#define CMD_SET_LAB             0x04    // payload[0] = lab_id

// --- Telemetry / Link (0x10–0x1F) ---
#define CMD_SET_FAST_HZ         0x10    // payload[0..1] = u16 hz (10–200)
#define CMD_SET_SLOW_HZ         0x11    // payload[0..1] = u16 hz (1–20)
#define CMD_SET_STREAM_MASK     0x12    // payload[0] = bitmask (bit0=FAST, bit1=SLOW)
#define CMD_SET_WATCHDOG_MS     0x13    // payload[0..1] = u16 ms (0=disable)

// --- Test Execution (0x20–0x2F) ---
#define CMD_RUN_TEST            0x20    // payload[0]=test_id, [1..8]=p1–p4 (i16 LE)
#define CMD_STOP                0x21    // soft stop (decelerate)
#define CMD_ABORT               0x22    // emergency stop + disarm
#define CMD_RESET_COUNTERS      0x23    // reset encoder / diag counters

// --- Runtime Tuning (0x40–0x4F) ---
// Each carries 5 × int16 (10 bytes) in payload[0..9]
#define CMD_SET_FF_LINEAR       0x40    // [bias_mV, speed×100, acc×100, tau_ms, rsvd]
#define CMD_SET_FF_ROTATION     0x41    // [bias_mV, speed×100, acc×100, tau_ms, rsvd]
#define CMD_SET_PD_LINEAR       0x42    // [Kp×100, Kd×100, maxOut_mV, rsvd, rsvd]
#define CMD_SET_PD_ROTATION     0x43    // [Kp×100, Kd×100, maxOut_mV, rsvd, rsvd]
#define CMD_SET_PD_STEERING     0x44    // [Kp×100, Kd×100, maxOut_mV, rsvd, rsvd]
#define CMD_SET_PD_WALL         0x45    // [Kp×100, Kd×100, maxOut_mV, rsvd, rsvd]

// ─────────────────────────────────────────────────────────────────────────────
// Test IDs  (CMD_RUN_TEST payload[0])
//   Parameters p1–p4 are int16 LE in payload[1..8]
// ─────────────────────────────────────────────────────────────────────────────

// Lab 1 — Hardware Check
#define TEST_MOTOR_SPIN         0x01    // p1=mV(+fwd/−rev), p2=duration_ms, p3=0=both/1=L/2=R
#define TEST_LED_SET            0x02    // p1=R, p2=G, p3=B (0–255)
#define TEST_BUZZER             0x03    // p1=freq_hz, p2=duration_ms
#define TEST_READ_SENSORS       0x04    // RSP: wallL,wallFL,wallFR,wallR (u16×4, mm)
#define TEST_READ_BATTERY       0x05    // RSP: vBat_mV(u16), raw_adc(u16)
#define TEST_READ_IMU           0x06    // RSP: gyroX,Y,Z(i16), accelX,Y,Z(i16) (raw)
#define TEST_READ_BUTTONS       0x07    // RSP: start(u8), mode(u8), dip(u8)
#define TEST_LED_BRIGHTNESS     0x08    // p1=brightness(0–255). RSP: current brightness
#define TEST_CAL_GYRO           0x09    // Calibrate gyro offsets (N samples). RSP: offsets
#define TEST_CAL_WALLS          0x0A    // Calibrate wall sensor offsets. RSP: offsets
#define TEST_CAL_QUERY          0x0B    // Query current calibration values (no measurement)
#define TEST_WALL_SET_OFFSET    0x0C    // Set wall sensor offset: p1=pos(0-3), p2=offset_i8

// Lab 2a — Encoder CPR
#define TEST_ENC_MONITOR        0x10    // start streaming abs counts via FAST override
#define TEST_ENC_RESET          0x11    // reset encoder accumulators to 0

// Lab 2b — Motor Characterization
#define TEST_RAMP_UP            0x20    // p1=start_mV, p2=end_mV, p3=step_mV, p4=step_ms
#define TEST_STEP_RESPONSE      0x21    // p1=mV, p2=duration_ms
#define TEST_FIND_BIAS          0x22    // auto-find min mV to move. RSP: biasL_mV, biasR_mV

// Lab 3 — Motion Profile
#define TEST_TRAP_FWD           0x30    // p1=distance_mm, p2=max_vel_mmps, p3=accel_mmps2
#define TEST_TRAP_TURN          0x31    // p1=angle_deg(+CW), p2=max_dps, p3=accel
#define TEST_SCURVE_FWD         0x32    // p1=distance_mm, p2=max_vel_mmps, p3=accel, p4=jerk_factor

// Lab 4 — PID Tuning
#define TEST_STRAIGHT           0x40    // p1=distance_mm, p2=vel_mmps
#define TEST_TURN_90            0x41    // p1=direction(+1=CW,−1=CCW)
#define TEST_TURN_180           0x42    // p1=direction
#define TEST_SMOOTH_ARC         0x43    // p1=radius_mm, p2=angle_deg, p3=vel_mmps
#define TEST_MOVE_CELLS         0x44    // p1=num_cells, p2=vel_mmps

// Lab 5 — Search Run
#define TEST_SEARCH_RUN         0x50    // payload[1]: mode 0=stop-go, 1=continuous
#define TEST_SEARCH_STEP        0x51    // single step (one cell, for debug)
#define TEST_SEARCH_RETURN      0x52    // return to start via shortest path
#define TEST_SEARCH_RESET       0x53    // reset maze + position
#define TEST_SEARCH_READ_WALLS  0x54    // read-only: returns wall sensor distances
#define TEST_SEARCH_QUERY       0x55    // read-only: returns pose + state (GUI polls)
#define TEST_SEARCH_MAZE_ROW    0x56    // read-only: returns wall data for 1 row
#define TEST_WALL_FOLLOW_L      0x57    // wall follow left-hand rule (continuous)
#define TEST_WALL_FOLLOW_R      0x58    // wall follow right-hand rule (continuous)

// ── Lab 6: Speed Run (0x60-0x6F) ────────────────────────────────────────────
#define TEST_SPEEDRUN_START     0x60    // payload[1]=speed_level(0-3)
#define TEST_SPEEDRUN_STOP      0x61    // abort speed run
#define TEST_SPEEDRUN_QUERY     0x62    // read-only: returns speed run state

// ─────────────────────────────────────────────────────────────────────────────
// RSP Status Codes (RSP byte[5])
// ─────────────────────────────────────────────────────────────────────────────
#define RSP_OK                  0x00
#define RSP_ERR_UNKNOWN_OP      0x01
#define RSP_ERR_BUSY            0x02    // queue full or test running
#define RSP_ERR_CRC             0x03
#define RSP_ERR_LEN             0x04    // packet not 20 bytes
#define RSP_ERR_BAD_ARG         0x05    // out-of-range parameter
#define RSP_ERR_DISARMED        0x06    // motor cmd while disarmed
#define RSP_ERR_LOCKED          0x07    // safety lockout / cfg pending

// ─────────────────────────────────────────────────────────────────────────────
// State Machine / Flags
// ─────────────────────────────────────────────────────────────────────────────

// Robot state (FastPacket.state lower nibble)
#define STATE_IDLE              0x00
#define STATE_ARMED             0x01
#define STATE_RUNNING           0x02
#define STATE_STOPPING          0x03
#define STATE_ERROR             0x04

// State flag bits (FastPacket.state upper nibble)
#define FLAG_ARMED              (1 << 4)
#define FLAG_RUNNING            (1 << 5)
#define FLAG_WATCHDOG           (1 << 6)
#define FLAG_LOW_BATT           (1 << 7)

// Active feature flags (SlowPacket.active_flags)
#define FEAT_MOTOR_OK           (1 << 0)
#define FEAT_ENCODER_OK         (1 << 1)
#define FEAT_IMU_OK             (1 << 2)
#define FEAT_WALL_SNS_OK        (1 << 3)
#define FEAT_BLE_CONN           (1 << 4)
#define FEAT_ARMED              (1 << 5)
#define FEAT_TEST_ACTIVE        (1 << 6)

// Stream mask bits (CMD_SET_STREAM_MASK)
#define STREAM_FAST             (1 << 0)
#define STREAM_SLOW             (1 << 1)

// Lab IDs (CMD_SET_LAB)
#define LAB_NONE                0
#define LAB_1_HW_CHECK          1
#define LAB_2A_ENC_CPR          2
#define LAB_2B_MOTOR_CHAR       3
#define LAB_3_MOTION_PROFILE    4
#define LAB_4_PID_TUNING        5
#define LAB_5_SEARCH_RUN        6
#define LAB_6_FAST_RUN          7

// ─────────────────────────────────────────────────────────────────────────────
// Wall Sensor Type (CAPS & config)
// ─────────────────────────────────────────────────────────────────────────────
#define WALL_SNS_NONE           0
#define WALL_SNS_IR_ANALOG      1       // SHARP GP2Y0A51SK0F (ADC)
#define WALL_SNS_VL6180X        2       // VL6180X (I2C ToF)
#define WALL_SNS_MIXED          3       // combination

// Sensor position index (fixed slots)
#define SNS_POS_LEFT            0
#define SNS_POS_FRONT_LEFT      1
#define SNS_POS_FRONT_RIGHT     2
#define SNS_POS_RIGHT           3
#define SNS_POS_MAX             4       // standard 4-sensor config (up to 6)

// ─────────────────────────────────────────────────────────────────────────────
// Scaling Contract  (scale_id = 1)
//
// Python must check CAPS.scale_id matches before interpreting tuning commands.
//
//  CMD_SET_FF_LINEAR / CMD_SET_FF_ROTATION:
//    params[5] = { bias_mV, speed_ff_x100, acc_ff_x100, tau_ms, reserved }
//    FW: bias_mV      → direct mV
//        speed_ff     → params[1] / 100.0   (mV per mm/s or mV per deg/s)
//        acc_ff       → params[2] / 100.0   (mV per mm/s² or mV per deg/s²)
//        tau_ms       → motor time constant (ms)
//
//  CMD_SET_PD_*:
//    params[5] = { Kp_x100, Kd_x100, max_output_mV, reserved, reserved }
//    FW: Kp           → params[0] / 100.0
//        Kd           → params[1] / 100.0
//        max_output   → direct mV clamp
//
//  Wire range: int16 → ±32767
//  FW must clamp after unscaling.
// ─────────────────────────────────────────────────────────────────────────────

// Tuning parameter indices (within 5-element i16 array)
#define TUNE_FF_BIAS_MV         0       // ×1   (mV)
#define TUNE_FF_SPEED_X100      1       // ×100 (mV per unit speed)
#define TUNE_FF_ACC_X100        2       // ×100 (mV per unit accel)
#define TUNE_FF_TAU_MS          3       // ×1   (ms)
#define TUNE_FF_RESERVED        4

#define TUNE_PD_KP_X100         0       // ×100
#define TUNE_PD_KD_X100         1       // ×100
#define TUNE_PD_MAX_MV          2       // ×1   (mV)
#define TUNE_PD_RESERVED_3      3
#define TUNE_PD_RESERVED_4      4

// ─────────────────────────────────────────────────────────────────────────────
// CRC8/ATM  —  poly=0x07 init=0x00 refin=false refout=false xorout=0x00
//   Used for ALL fixed 20B packets. CRC over bytes[0..18], result at byte[19].
// ─────────────────────────────────────────────────────────────────────────────
static inline uint8_t proto_crc8(const uint8_t* data, size_t len) {
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07)
                               : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

// ─────────────────────────────────────────────────────────────────────────────
// CRC16/CCITT-FALSE  —  for CFG variable frames (future)
//   poly=0x1021 init=0xFFFF
// ─────────────────────────────────────────────────────────────────────────────
static inline uint16_t proto_crc16(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int b = 0; b < 8; b++) {
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021)
                                 : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

// ═══════════════════════════════════════════════════════════════════════════════
// Packet Structures — ALL exactly 20 bytes, packed, CRC8 at byte[19]
// ═══════════════════════════════════════════════════════════════════════════════

#pragma pack(push, 1)

// ── CAPS Packet (READ characteristic) ────────────────────────────────────────
//    FW fills once at boot. Python reads to verify compatibility.
typedef struct {
    uint8_t  type;              // [0]  = PKT_CAPS (0x10)
    uint8_t  proto_ver;         // [1]  = PROTO_VER
    uint16_t fw_ver;            // [2-3]  (major<<8 | minor)
    uint8_t  fast_size;         // [4]  = sizeof(FastPacket) must be 20
    uint8_t  slow_size;         // [5]  = sizeof(SlowPacket) must be 20
    uint8_t  cmd_size;          // [6]  = sizeof(CmdPacket)  must be 20
    uint8_t  rsp_size;          // [7]  = sizeof(RspPacket)  must be 20
    uint16_t features;          // [8-9]  feature bitmask
    uint8_t  scale_id;          // [10] = SCALE_ID
    uint8_t  num_wall_sensors;  // [11] 0–6
    uint8_t  wall_sensor_type;  // [12] WALL_SNS_*
    uint8_t  num_labs;          // [13] how many labs FW supports
    uint8_t  max_fast_hz;       // [14] max FAST rate FW supports
    uint8_t  max_slow_hz;       // [15] max SLOW rate
    uint8_t  reserved[3];       // [16-18] must be 0
    uint8_t  crc8;              // [19]
} CapsPacket;

// ── CMD Packet (WRITE_NR characteristic) ─────────────────────────────────────
//    Host → FW.  BLE callback copies + enqueues. No parse in callback.
typedef struct {
    uint8_t  type;              // [0]  = PKT_CMD (0x20)
    uint8_t  seq;               // [1]  host sequence (mod 256)
    uint8_t  opcode;            // [2]  CMD_*
    uint16_t req_id;            // [3-4]  request ID for RSP matching (LE)
    uint8_t  payload[14];       // [5-18] command-specific data
    uint8_t  crc8;              // [19]
} CmdPacket;

// ── RSP Packet (NOTIFY characteristic) ───────────────────────────────────────
//    FW → Host.  Sent by CommsTask (Core0) ONLY. One RSP per req_id.
typedef struct {
    uint8_t  type;              // [0]  = PKT_RSP (0x30)
    uint8_t  seq;               // [1]  FW sequence
    uint8_t  opcode;            // [2]  echo of CMD opcode
    uint16_t req_id;            // [3-4]  echo of CMD req_id
    uint8_t  status;            // [5]  RSP_*
    uint8_t  payload[13];       // [6-18] optional response data
    uint8_t  crc8;              // [19]
} RspPacket;

// ── FAST Packet (NOTIFY characteristic) ──────────────────────────────────────
//    FW → Host.  Real-time motor telemetry. 50–200 Hz.
//    Control Core1 fills SharedSnapshot → CommsTask Core0 packs + notifies.
typedef struct {
    uint8_t  type;              // [0]  = PKT_FAST (0x01)
    uint8_t  seq;               // [1]
    uint16_t t_ms;              // [2-3]  millis() mod 65536
    int16_t  vCmdL_mV;         // [4-5]  left motor voltage command
    int16_t  vCmdR_mV;         // [6-7]  right motor voltage command
    int16_t  velL_mmps;        // [8-9]  left wheel velocity (mm/s)
    int16_t  velR_mmps;        // [10-11] right wheel velocity (mm/s)
    int16_t  v_target_mmps;    // [12-13] motion profile target velocity (mm/s)
    int16_t  gyroZ_dps10;      // [14-15] gyro Z × 10 (0.1 °/s resolution)
    int8_t   encL_delta;       // [16] left encoder delta (counts/tick)
    int8_t   encR_delta;       // [17] right encoder delta (counts/tick)
    uint8_t  state;            // [18] STATE_* | FLAG_* bits
    uint8_t  crc8;             // [19]
} FastPacket;

// ── SLOW Packet (NOTIFY characteristic) ──────────────────────────────────────
//    FW → Host.  Diagnostics + health. 5–20 Hz.
typedef struct {
    uint8_t  type;              // [0]  = PKT_SLOW (0x02)
    uint8_t  seq;               // [1]
    uint16_t t_ms;              // [2-3]
    uint16_t missed_ticks;      // [4-5]  control overruns (accumulated)
    uint16_t rx_drop_count;     // [6-7]  BLE RX drops (queue full + bad len)
    uint16_t crc_fail_count;    // [8-9]  CRC failures
    uint8_t  last_cmd_op;       // [10] last applied CMD opcode
    uint8_t  last_cmd_status;   // [11] its RSP status
    uint16_t wd_age_ms;         // [12-13] ms since last valid CMD (watchdog)
    uint16_t step_max_us;       // [14-15] peak 1kHz step exec time (µs)
    uint16_t vBat_avg_mV;       // [16-17] filtered battery voltage
    uint8_t  active_flags;      // [18] FEAT_* bitmask
    uint8_t  crc8;              // [19]
} SlowPacket;

#pragma pack(pop)

// ═══════════════════════════════════════════════════════════════════════════════
// Compile-time Size Checks — CRITICAL, do not remove
// ═══════════════════════════════════════════════════════════════════════════════
#ifdef __cplusplus
static_assert(sizeof(CapsPacket) == PROTO_PACKET_SIZE, "CapsPacket != 20");
static_assert(sizeof(CmdPacket)  == PROTO_PACKET_SIZE, "CmdPacket  != 20");
static_assert(sizeof(RspPacket)  == PROTO_PACKET_SIZE, "RspPacket  != 20");
static_assert(sizeof(FastPacket) == PROTO_PACKET_SIZE, "FastPacket != 20");
static_assert(sizeof(SlowPacket) == PROTO_PACKET_SIZE, "SlowPacket != 20");
#else
_Static_assert(sizeof(CapsPacket) == PROTO_PACKET_SIZE, "CapsPacket != 20");
_Static_assert(sizeof(CmdPacket)  == PROTO_PACKET_SIZE, "CmdPacket  != 20");
_Static_assert(sizeof(RspPacket)  == PROTO_PACKET_SIZE, "RspPacket  != 20");
_Static_assert(sizeof(FastPacket) == PROTO_PACKET_SIZE, "FastPacket != 20");
_Static_assert(sizeof(SlowPacket) == PROTO_PACKET_SIZE, "SlowPacket != 20");
#endif

// ═══════════════════════════════════════════════════════════════════════════════
// Mailbox / Command Queue Types (shared between comms & control)
// ═══════════════════════════════════════════════════════════════════════════════

// CommandRequest: what gets enqueued into g_cmdQ
typedef struct {
    uint8_t  opcode;            // CMD_*
    uint16_t req_id;            // for RSP matching
    uint8_t  payload[14];       // verbatim copy from CmdPacket.payload
} CommandRequest;

// CommandResult: what control returns after apply
typedef struct {
    uint8_t  opcode;
    uint16_t req_id;
    uint8_t  status;            // RSP_*
    uint8_t  rsp_payload[13];   // optional data for RSP
    uint8_t  rsp_payload_len;   // how many bytes of rsp_payload are valid
} CommandResult;

// ═══════════════════════════════════════════════════════════════════════════════
// SharedSnapshot — Core1 writes, Core0 reads (protected by portMUX)
//   Separate Fast and Slow sections, both under SAME spinlock.
// ═══════════════════════════════════════════════════════════════════════════════

typedef struct {
    // ── Fast section (updated every 1kHz tick) ──
    uint16_t t_ms;
    int16_t  vCmdL_mV;
    int16_t  vCmdR_mV;
    int16_t  velL_mmps;
    int16_t  velR_mmps;
    int16_t  v_target_mmps;    // motion profile target velocity (mm/s)
    int16_t  gyroZ_dps10;
    int8_t   encL_delta;
    int8_t   encR_delta;
    uint8_t  state;

    // ── Slow section (updated at lower rate or on events) ──
    uint16_t missed_ticks;
    uint16_t step_max_us;
    uint16_t vBat_avg_mV;
    uint8_t  last_cmd_op;
    uint8_t  last_cmd_status;
    uint8_t  active_flags;
} SharedSnapshot;

// ═══════════════════════════════════════════════════════════════════════════════
// Packet Build / Validate Helpers
// ═══════════════════════════════════════════════════════════════════════════════

// Stamp CRC8 at byte[19] for any 20-byte packet
static inline void proto_stamp_crc(void* pkt) {
    uint8_t* p = (uint8_t*)pkt;
    p[19] = proto_crc8(p, 19);
}

// Validate: type matches + CRC8 correct
static inline int proto_validate(const void* pkt, uint8_t expected_type) {
    const uint8_t* p = (const uint8_t*)pkt;
    if (p[0] != expected_type) return 0;
    return (proto_crc8(p, 19) == p[19]);
}

// Generic validate (any type, just check CRC)
static inline int proto_validate_crc(const void* pkt) {
    const uint8_t* p = (const uint8_t*)pkt;
    return (proto_crc8(p, 19) == p[19]);
}

// Build CAPS packet (call once at boot)
static inline void proto_build_caps(CapsPacket* c,
                                     uint8_t num_wall, uint8_t wall_type,
                                     uint8_t num_labs) {
    if (!c) return;
    uint8_t* p = (uint8_t*)c;
    for (int i = 0; i < 20; i++) p[i] = 0;  // zero-fill

    c->type             = PKT_CAPS;
    c->proto_ver        = PROTO_VER;
    c->fw_ver           = FW_VER;
    c->fast_size        = sizeof(FastPacket);
    c->slow_size        = sizeof(SlowPacket);
    c->cmd_size         = sizeof(CmdPacket);
    c->rsp_size         = sizeof(RspPacket);
    c->features         = 0;          // set by caller
    c->scale_id         = SCALE_ID;
    c->num_wall_sensors = num_wall;
    c->wall_sensor_type = wall_type;
    c->num_labs         = num_labs;
    c->max_fast_hz      = 200;
    c->max_slow_hz      = 20;
    proto_stamp_crc(c);
}

// Build RSP from a CommandResult
static inline void proto_build_rsp(RspPacket* r, uint8_t seq,
                                    const CommandResult* res) {
    if (!r || !res) return;
    uint8_t* p = (uint8_t*)r;
    for (int i = 0; i < 20; i++) p[i] = 0;

    r->type    = PKT_RSP;
    r->seq     = seq;
    r->opcode  = res->opcode;
    r->req_id  = res->req_id;
    r->status  = res->status;
    // copy response payload
    uint8_t n = res->rsp_payload_len;
    if (n > 13) n = 13;
    for (uint8_t i = 0; i < n; i++) {
        r->payload[i] = res->rsp_payload[i];
    }
    proto_stamp_crc(r);
}

// Pack SharedSnapshot into FastPacket
static inline void proto_build_fast(FastPacket* f, uint8_t seq,
                                     const SharedSnapshot* ss) {
    if (!f || !ss) return;
    f->type         = PKT_FAST;
    f->seq          = seq;
    f->t_ms         = ss->t_ms;
    f->vCmdL_mV    = ss->vCmdL_mV;
    f->vCmdR_mV    = ss->vCmdR_mV;
    f->velL_mmps   = ss->velL_mmps;
    f->velR_mmps   = ss->velR_mmps;
    f->v_target_mmps = ss->v_target_mmps;
    f->gyroZ_dps10 = ss->gyroZ_dps10;
    f->encL_delta  = ss->encL_delta;
    f->encR_delta  = ss->encR_delta;
    f->state       = ss->state;
    proto_stamp_crc(f);
}

// Pack SharedSnapshot into SlowPacket
static inline void proto_build_slow(SlowPacket* s, uint8_t seq,
                                     const SharedSnapshot* ss,
                                     uint16_t rx_drops, uint16_t crc_fails,
                                     uint16_t wd_age_ms) {
    if (!s || !ss) return;
    s->type             = PKT_SLOW;
    s->seq              = seq;
    s->t_ms             = ss->t_ms;
    s->missed_ticks     = ss->missed_ticks;
    s->rx_drop_count    = rx_drops;
    s->crc_fail_count   = crc_fails;
    s->last_cmd_op      = ss->last_cmd_op;
    s->last_cmd_status  = ss->last_cmd_status;
    s->wd_age_ms        = wd_age_ms;
    s->step_max_us      = ss->step_max_us;
    s->vBat_avg_mV      = ss->vBat_avg_mV;
    s->active_flags     = ss->active_flags;
    proto_stamp_crc(s);
}

// ─────────────────────────────────────────────────────────────────────────────
// Clamping helpers
// ─────────────────────────────────────────────────────────────────────────────
static inline int16_t proto_clamp_i16(int16_t v, int16_t lo, int16_t hi) {
    return (v < lo) ? lo : (v > hi) ? hi : v;
}
static inline uint16_t proto_clamp_u16(uint16_t v, uint16_t lo, uint16_t hi) {
    return (v < lo) ? lo : (v > hi) ? hi : v;
}

#ifdef __cplusplus
} // extern "C"
#endif
