"""
mm_protocol.py — Micromouse BLE Protocol v2.0 Codec
Mirrors protocol.h exactly. Single source of truth for Python side.
"""
import struct

# ═══════════════════════════════════════════════════════════════════════════════
# Constants
# ═══════════════════════════════════════════════════════════════════════════════
PROTO_VER = 2
PKT_SIZE = 20

PKT_FAST = 0x01
PKT_SLOW = 0x02
PKT_CAPS = 0x10
PKT_CMD  = 0x20
PKT_RSP  = 0x30

# GATT UUIDs
UUID_SERVICE = "a0000000-5a19-40b6-926c-f22d00d00000"
UUID_CAPS    = "a0000001-5a19-40b6-926c-f22d00d00000"
UUID_CMD     = "a0000002-5a19-40b6-926c-f22d00d00000"
UUID_RSP     = "a0000003-5a19-40b6-926c-f22d00d00000"
UUID_FAST    = "a0000004-5a19-40b6-926c-f22d00d00000"
UUID_SLOW    = "a0000005-5a19-40b6-926c-f22d00d00000"

# CMD opcodes
CMD_PING            = 0x01
CMD_ARM             = 0x02
CMD_DISARM          = 0x03
CMD_SET_LAB         = 0x04
CMD_SET_FAST_HZ     = 0x10
CMD_SET_SLOW_HZ     = 0x11
CMD_SET_STREAM_MASK = 0x12
CMD_SET_WATCHDOG_MS = 0x13
CMD_RUN_TEST        = 0x20
CMD_STOP            = 0x21
CMD_ABORT           = 0x22
CMD_RESET_COUNTERS  = 0x23

# Test IDs
TEST_MOTOR_SPIN     = 0x01
TEST_LED_SET        = 0x02
TEST_BUZZER         = 0x03
TEST_READ_SENSORS   = 0x04
TEST_READ_BATTERY   = 0x05
TEST_READ_IMU       = 0x06
TEST_READ_BUTTONS   = 0x07
TEST_LED_BRIGHTNESS = 0x08
TEST_CAL_GYRO       = 0x09
TEST_CAL_WALLS      = 0x0A
TEST_CAL_QUERY      = 0x0B
TEST_WALL_SET_OFFSET = 0x0C

# Lab 2a: Encoder
TEST_ENC_MONITOR    = 0x10
TEST_ENC_RESET      = 0x11

# Lab 2b: Motor Characterization
TEST_RAMP_UP        = 0x20
TEST_STEP_RESPONSE  = 0x21
TEST_FIND_BIAS      = 0x22

# Lab 3: Motion Profile
TEST_TRAP_FWD       = 0x30
TEST_TRAP_TURN      = 0x31

# Lab 4: PID-Controlled Motion
TEST_STRAIGHT       = 0x40
TEST_TURN_90        = 0x41
TEST_TURN_180       = 0x42
TEST_SMOOTH_ARC     = 0x43
TEST_MOVE_CELLS     = 0x44

# RSP status
RSP_OK              = 0x00
RSP_ERR_UNKNOWN_OP  = 0x01
RSP_ERR_BUSY        = 0x02
RSP_ERR_CRC         = 0x03
RSP_ERR_LEN         = 0x04
RSP_ERR_BAD_ARG     = 0x05
RSP_ERR_DISARMED    = 0x06
RSP_ERR_LOCKED      = 0x07

RSP_NAMES = {
    0x00: "OK", 0x01: "UNKNOWN_OP", 0x02: "BUSY", 0x03: "CRC",
    0x04: "LEN", 0x05: "BAD_ARG", 0x06: "DISARMED", 0x07: "LOCKED",
}

# State flags
FLAG_ARMED    = 0x10
FLAG_RUNNING  = 0x20
FLAG_WATCHDOG = 0x40
FLAG_LOW_BATT = 0x80


# ═══════════════════════════════════════════════════════════════════════════════
# CRC8/ATM — poly=0x07 init=0x00
# ═══════════════════════════════════════════════════════════════════════════════
def crc8(data: bytes) -> int:
    crc = 0x00
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc

def verify_crc(pkt: bytes) -> bool:
    return len(pkt) == PKT_SIZE and crc8(pkt[:19]) == pkt[19]


# ═══════════════════════════════════════════════════════════════════════════════
# CMD Builder
# ═══════════════════════════════════════════════════════════════════════════════
_seq_counter = 0

def build_cmd(opcode: int, req_id: int, payload: bytes = b'') -> bytes:
    global _seq_counter
    buf = bytearray(PKT_SIZE)
    buf[0] = PKT_CMD
    buf[1] = _seq_counter & 0xFF
    _seq_counter += 1
    buf[2] = opcode
    struct.pack_into('<H', buf, 3, req_id)
    n = min(len(payload), 14)
    buf[5:5+n] = payload[:n]
    buf[19] = crc8(bytes(buf[:19]))
    return bytes(buf)

def cmd_ping(req_id=1):
    return build_cmd(CMD_PING, req_id)

def cmd_arm(req_id=2):
    return build_cmd(CMD_ARM, req_id)

def cmd_disarm(req_id=3):
    return build_cmd(CMD_DISARM, req_id)

def cmd_stop(req_id=4):
    return build_cmd(CMD_STOP, req_id)

def cmd_abort(req_id=5):
    return build_cmd(CMD_ABORT, req_id)

def cmd_set_fast_hz(hz, req_id=10):
    return build_cmd(CMD_SET_FAST_HZ, req_id, struct.pack('<H', hz))

def cmd_set_slow_hz(hz, req_id=11):
    return build_cmd(CMD_SET_SLOW_HZ, req_id, struct.pack('<H', hz))

def cmd_set_stream_mask(mask, req_id=12):
    return build_cmd(CMD_SET_STREAM_MASK, req_id, bytes([mask]))

def cmd_set_watchdog(ms, req_id=13):
    return build_cmd(CMD_SET_WATCHDOG_MS, req_id, struct.pack('<H', ms))

def cmd_reset_counters(req_id=14):
    return build_cmd(CMD_RESET_COUNTERS, req_id)

def cmd_run_test(test_id, p1=0, p2=0, p3=0, p4=0, req_id=100):
    """Build RUN_TEST: payload[0]=test_id, [1..8]=p1-p4 as int16 LE"""
    payload = struct.pack('<BhhhH', test_id, p1, p2, p3, p4)
    return build_cmd(CMD_RUN_TEST, req_id, payload)

def cmd_motor_spin(mV, duration_ms, select=0, req_id=101):
    """Motor spin: mV(+fwd/-rev), duration(ms), select(0=both,1=L,2=R)"""
    payload = struct.pack('<BhhB', TEST_MOTOR_SPIN, mV, duration_ms, select)
    # pad to include select in correct position (payload[5])
    full = bytearray(14)
    full[0] = TEST_MOTOR_SPIN
    struct.pack_into('<h', full, 1, mV)
    struct.pack_into('<H', full, 3, duration_ms)
    full[5] = select
    return build_cmd(CMD_RUN_TEST, req_id, bytes(full))

def cmd_led_set(r, g, b, req_id=102):
    full = bytearray(14)
    full[0] = TEST_LED_SET
    struct.pack_into('<h', full, 1, r)
    struct.pack_into('<h', full, 3, g)
    struct.pack_into('<h', full, 5, b)
    return build_cmd(CMD_RUN_TEST, req_id, bytes(full))

def cmd_buzzer(freq, duration_ms, req_id=103):
    full = bytearray(14)
    full[0] = TEST_BUZZER
    struct.pack_into('<H', full, 1, freq)
    struct.pack_into('<H', full, 3, duration_ms)
    return build_cmd(CMD_RUN_TEST, req_id, bytes(full))

def cmd_read_sensors(req_id=104):
    return build_cmd(CMD_RUN_TEST, req_id, bytes([TEST_READ_SENSORS]))

def cmd_read_battery(req_id=105):
    return build_cmd(CMD_RUN_TEST, req_id, bytes([TEST_READ_BATTERY]))

def cmd_read_imu(req_id=106):
    return build_cmd(CMD_RUN_TEST, req_id, bytes([TEST_READ_IMU]))

def cmd_read_buttons(req_id=107):
    return build_cmd(CMD_RUN_TEST, req_id, bytes([TEST_READ_BUTTONS]))

# ── Calibration ──────────────────────────────────────────────────────────────

def cmd_cal_gyro(samples_div10=50, req_id=108):
    """Calibrate gyro offsets. Robot must be stationary!
    samples_div10: actual samples = value × 10 (50→500 samples ~1s)
    RSP: gyroZ_off×100(i16) gyroX_off×100(i16) gyroY_off×100(i16) calibrated(u8)"""
    payload = bytearray(10)
    payload[0] = 0x09  # TEST_CAL_GYRO
    payload[1] = samples_div10 & 0xFF
    return build_cmd(CMD_RUN_TEST, req_id, bytes(payload))

def cmd_cal_walls(samples_div10=5, req_id=109):
    """Measure wall sensor raw averages (does NOT change offsets).
    samples_div10: actual samples = value × 10 (5→50 samples ~1.25s)
    RSP: raw_L(u16) raw_FL(u16) raw_FR(u16) raw_R(u16) offsets(i8×4)"""
    payload = bytearray(10)
    payload[0] = 0x0A  # TEST_CAL_WALLS
    payload[1] = samples_div10 & 0xFF
    return build_cmd(CMD_RUN_TEST, req_id, bytes(payload))

def cmd_wall_set_offset(position, offset_mm, req_id=111):
    """Set offset for one wall sensor.
    position: 0=L, 1=FL, 2=FR, 3=R
    offset_mm: signed int8 (-127 to +127)"""
    payload = bytearray(10)
    payload[0] = 0x0C  # TEST_WALL_SET_OFFSET
    payload[1] = position & 0x03
    payload[2] = offset_mm & 0xFF  # signed as unsigned byte
    return build_cmd(CMD_RUN_TEST, req_id, bytes(payload))

def cmd_cal_query(req_id=110):
    """Query current calibration values (no measurement)."""
    payload = bytearray(10)
    payload[0] = 0x0B  # TEST_CAL_QUERY
    return build_cmd(CMD_RUN_TEST, req_id, bytes(payload))

# ── Lab 2a: Encoder ──────────────────────────────────────────────────────────

def cmd_enc_monitor(req_id=110):
    """Read absolute encoder counts L/R."""
    return build_cmd(CMD_RUN_TEST, req_id, bytes([TEST_ENC_MONITOR]))

def cmd_enc_reset(req_id=111):
    """Reset encoder accumulators to 0."""
    return build_cmd(CMD_RUN_TEST, req_id, bytes([TEST_ENC_RESET]))

def parse_rsp_enc_monitor(payload: bytes) -> dict:
    """Parse ENC_MONITOR RSP: countL(i32) + countR(i32)"""
    return {
        'countL': struct.unpack_from('<i', payload, 0)[0],
        'countR': struct.unpack_from('<i', payload, 4)[0],
    }

# ── Lab 2b: Motor Characterization ──────────────────────────────────────────

def cmd_step_response(mV, duration_ms, select=0, req_id=121):
    """Step response: constant mV for duration. select: 0=both,1=L,2=R"""
    full = bytearray(14)
    full[0] = TEST_STEP_RESPONSE
    struct.pack_into('<h', full, 1, mV)
    struct.pack_into('<H', full, 3, duration_ms)
    full[5] = select
    return build_cmd(CMD_RUN_TEST, req_id, bytes(full))

def cmd_ramp_up(start_mV, end_mV, step_mV, step_ms, select=0, req_id=120):
    """Ramp: sweep voltage from start→end. step_ms = dwell time per step."""
    full = bytearray(14)
    full[0] = TEST_RAMP_UP
    struct.pack_into('<h', full, 1, start_mV)
    struct.pack_into('<h', full, 3, end_mV)
    struct.pack_into('<h', full, 5, step_mV)
    struct.pack_into('<H', full, 7, step_ms)
    full[9] = select
    return build_cmd(CMD_RUN_TEST, req_id, bytes(full))

def cmd_find_bias(req_id=122):
    """Auto-find minimum voltage to start moving (both motors sequentially)."""
    return build_cmd(CMD_RUN_TEST, req_id, bytes([TEST_FIND_BIAS]))

# ── LED Brightness ───────────────────────────────────────────────────────────

def cmd_led_brightness(brightness, req_id=108):
    """Set LED brightness 0-255."""
    full = bytearray(14)
    full[0] = TEST_LED_BRIGHTNESS
    struct.pack_into('<h', full, 1, brightness)
    return build_cmd(CMD_RUN_TEST, req_id, bytes(full))

# ── Lab 3: Motion Profile ────────────────────────────────────────────────────

def cmd_trap_fwd(distance_mm, max_vel_mmps, accel_mmps2, req_id=130):
    """Trapezoidal straight: distance_mm (signed), max velocity, acceleration."""
    full = bytearray(14)
    full[0] = TEST_TRAP_FWD
    struct.pack_into('<h', full, 1, distance_mm)
    struct.pack_into('<h', full, 3, max_vel_mmps)
    struct.pack_into('<h', full, 5, accel_mmps2)
    return build_cmd(CMD_RUN_TEST, req_id, bytes(full))

def cmd_trap_turn(angle_deg, max_dps, accel_dps2, req_id=131):
    """Trapezoidal turn: angle_deg (signed +CW), max angular vel, acceleration."""
    full = bytearray(14)
    full[0] = TEST_TRAP_TURN
    struct.pack_into('<h', full, 1, angle_deg)
    struct.pack_into('<h', full, 3, max_dps)
    struct.pack_into('<h', full, 5, accel_dps2)
    return build_cmd(CMD_RUN_TEST, req_id, bytes(full))

# ── Lab 4: PID Motion ────────────────────────────────────────────────────────

def cmd_pid_straight(distance_mm, vel_mmps, accel_mmps2, req_id=140):
    """PID-controlled straight: distance(signed), velocity, acceleration."""
    full = bytearray(14)
    full[0] = TEST_STRAIGHT
    struct.pack_into('<h', full, 1, distance_mm)
    struct.pack_into('<h', full, 3, vel_mmps)
    struct.pack_into('<h', full, 5, accel_mmps2)
    return build_cmd(CMD_RUN_TEST, req_id, bytes(full))

def cmd_pid_turn_90(direction=1, req_id=141):
    """PID 90° turn. direction: +1=CW, -1=CCW."""
    full = bytearray(14)
    full[0] = TEST_TURN_90
    struct.pack_into('<h', full, 1, direction)
    return build_cmd(CMD_RUN_TEST, req_id, bytes(full))

def cmd_pid_turn_180(direction=1, req_id=142):
    """PID 180° turn. direction: +1=CW, -1=CCW."""
    full = bytearray(14)
    full[0] = TEST_TURN_180
    struct.pack_into('<h', full, 1, direction)
    return build_cmd(CMD_RUN_TEST, req_id, bytes(full))

def cmd_pid_move_cells(n_cells, vel_mmps, req_id=144):
    """PID straight N cells. n_cells signed (neg = backward)."""
    full = bytearray(14)
    full[0] = TEST_MOVE_CELLS
    struct.pack_into('<h', full, 1, n_cells)
    struct.pack_into('<h', full, 3, vel_mmps)
    return build_cmd(CMD_RUN_TEST, req_id, bytes(full))

# ── Runtime PID Tuning ───────────────────────────────────────────────────────
CMD_SET_PD_LINEAR   = 0x42
CMD_SET_PD_STEERING = 0x44

def cmd_set_pd_linear(kp_x100, kd_x100, max_mv, req_id=242):
    """Set velocity PID gains. Kp/Kd in ×100 integer (e.g. 550 = 5.50)."""
    payload = bytearray(10)
    struct.pack_into('<h', payload, 0, kp_x100)
    struct.pack_into('<h', payload, 2, kd_x100)
    struct.pack_into('<h', payload, 4, max_mv)
    return build_cmd(CMD_SET_PD_LINEAR, req_id, bytes(payload))

def cmd_set_pd_steering(kp_x100, ki_x100, kd_x100, max_mv, trim_mv=0, req_id=244):
    """Set heading PID gains + steering trim. All ×100 integers. trim_mv in raw mV."""
    payload = bytearray(10)
    struct.pack_into('<h', payload, 0, kp_x100)
    struct.pack_into('<h', payload, 2, ki_x100)
    struct.pack_into('<h', payload, 4, kd_x100)
    struct.pack_into('<h', payload, 6, max_mv)
    struct.pack_into('<h', payload, 8, trim_mv)
    return build_cmd(CMD_SET_PD_STEERING, req_id, bytes(payload))


# ── Lab 5: Maze Search ──────────────────────────────────────────────────────

def _embed_config(payload, cfg):
    """Embed goal/start config in payload bytes [2-9].
    cfg: dict with start_x, start_y, start_heading, goal_x_min/max, goal_y_min/max
    If cfg is None, bytes stay 0 (no config marker)."""
    if cfg is None:
        return
    payload[2] = 0xCF  # magic marker: "ConFig present"
    payload[3] = cfg.get('start_x', 0) & 0x0F
    payload[4] = cfg.get('start_y', 0) & 0x0F
    payload[5] = cfg.get('start_heading', 0) & 0x03
    payload[6] = cfg.get('goal_x_min', 7) & 0x0F
    payload[7] = cfg.get('goal_y_min', 7) & 0x0F
    payload[8] = cfg.get('goal_x_max', 8) & 0x0F
    payload[9] = cfg.get('goal_y_max', 8) & 0x0F

def cmd_search_run(mode=0, req_id=150, cfg=None):
    """Start full autonomous search. mode: 0=stop-and-go, 1=continuous.
    cfg: optional dict with goal/start config to embed."""
    payload = bytearray(10)
    payload[0] = 0x50  # TEST_SEARCH_RUN
    payload[1] = mode & 0xFF
    _embed_config(payload, cfg)
    return build_cmd(CMD_RUN_TEST, req_id, bytes(payload))

def cmd_search_step(req_id=151, cfg=None):
    """Single step — advance one cell (debug mode)."""
    payload = bytearray(10)
    payload[0] = 0x51  # TEST_SEARCH_STEP
    _embed_config(payload, cfg)
    return build_cmd(CMD_RUN_TEST, req_id, bytes(payload))

def cmd_search_return(req_id=152):
    """Return to start via shortest known path."""
    payload = bytearray(10)
    payload[0] = 0x52  # TEST_SEARCH_RETURN
    return build_cmd(CMD_RUN_TEST, req_id, bytes(payload))

def cmd_search_reset(req_id=153, cfg=None):
    """Reset maze + robot position."""
    payload = bytearray(10)
    payload[0] = 0x53  # TEST_SEARCH_RESET
    _embed_config(payload, cfg)
    return build_cmd(CMD_RUN_TEST, req_id, bytes(payload))

def cmd_search_read_walls(req_id=154):
    """Read wall sensors (one-shot, no ARM required).
    RSP payload: L(i16) FL(i16) FR(i16) R(i16) flags(u8) hdg(u8) x(u8) y(u8)"""
    payload = bytearray(10)
    payload[0] = 0x54  # TEST_SEARCH_READ_WALLS
    return build_cmd(CMD_RUN_TEST, req_id, bytes(payload))

def cmd_search_query(req_id=155):
    """Query search state + robot pose (no ARM required, GUI polls this)."""
    payload = bytearray(10)
    payload[0] = 0x55  # TEST_SEARCH_QUERY
    return build_cmd(CMD_RUN_TEST, req_id, bytes(payload))

def cmd_search_maze_row(row, req_id=156):
    """Get wall data for one maze row (no ARM required).
    row: 0-15. Returns packed wall nibbles + visited bitmask."""
    payload = bytearray(10)
    payload[0] = 0x56  # TEST_SEARCH_MAZE_ROW
    payload[1] = row & 0x0F
    return build_cmd(CMD_RUN_TEST, req_id, bytes(payload))

def cmd_wall_follow_left(req_id=157):
    """Start left-hand wall follower (continuous, needs ARM)."""
    payload = bytearray(10)
    payload[0] = 0x57  # TEST_WALL_FOLLOW_L
    return build_cmd(CMD_RUN_TEST, req_id, bytes(payload))

def cmd_wall_follow_right(req_id=158):
    """Start right-hand wall follower (continuous, needs ARM)."""
    payload = bytearray(10)
    payload[0] = 0x58  # TEST_WALL_FOLLOW_R
    return build_cmd(CMD_RUN_TEST, req_id, bytes(payload))

def cmd_search_set_config(start_x=0, start_y=0, start_heading=0,
                          goal_x_min=7, goal_y_min=7, goal_x_max=8, goal_y_max=8,
                          req_id=159):
    """Set search start/goal config before running search.
    start_heading: 0=N, 1=E, 2=S, 3=W
    goal: rectangle from (x_min,y_min) to (x_max,y_max)
    """
    payload = bytearray(10)
    payload[0] = 0x59  # TEST_SEARCH_SET_CONFIG
    payload[1] = start_x & 0x0F
    payload[2] = start_y & 0x0F
    payload[3] = start_heading & 0x03
    payload[4] = goal_x_min & 0x0F
    payload[5] = goal_y_min & 0x0F
    payload[6] = goal_x_max & 0x0F
    payload[7] = goal_y_max & 0x0F
    return build_cmd(CMD_RUN_TEST, req_id, bytes(payload))

# ── Lab 6: Speed Run ─────────────────────────────────────────────────────────

def cmd_speedrun_start(speed_level=0, req_id=160):
    """Start speed run. speed_level: 0=cautious, 1=moderate, 2=fast, 3=fastest."""
    payload = bytearray(10)
    payload[0] = 0x60  # TEST_SPEEDRUN_START
    payload[1] = speed_level & 0x03
    return build_cmd(CMD_RUN_TEST, req_id, bytes(payload))

def cmd_speedrun_stop(req_id=161):
    """Abort speed run."""
    payload = bytearray(10)
    payload[0] = 0x61
    return build_cmd(CMD_RUN_TEST, req_id, bytes(payload))

def cmd_speedrun_query(req_id=162):
    """Query speed run state (no ARM required)."""
    payload = bytearray(10)
    payload[0] = 0x62
    return build_cmd(CMD_RUN_TEST, req_id, bytes(payload))


# ═══════════════════════════════════════════════════════════════════════════════
# Packet Parsers
# ═══════════════════════════════════════════════════════════════════════════════

def parse_caps(data: bytes) -> dict:
    if not verify_crc(data): return None
    return {
        'proto_ver': data[1], 'fw_ver': struct.unpack_from('<H', data, 2)[0],
        'fast_size': data[4], 'slow_size': data[5],
        'cmd_size': data[6], 'rsp_size': data[7],
        'features': struct.unpack_from('<H', data, 8)[0],
        'scale_id': data[10], 'num_wall_sensors': data[11],
        'wall_sensor_type': data[12], 'num_labs': data[13],
        'max_fast_hz': data[14], 'max_slow_hz': data[15],
    }

def parse_rsp(data: bytes) -> dict:
    if not verify_crc(data): return None
    return {
        'type': data[0], 'seq': data[1], 'opcode': data[2],
        'req_id': struct.unpack_from('<H', data, 3)[0],
        'status': data[5], 'payload': data[6:19],
    }

def parse_fast(data: bytes) -> dict:
    if not verify_crc(data): return None
    return {
        'type': data[0], 'seq': data[1],
        't_ms':      struct.unpack_from('<H', data, 2)[0],
        'vCmdL_mV':  struct.unpack_from('<h', data, 4)[0],
        'vCmdR_mV':  struct.unpack_from('<h', data, 6)[0],
        'velL_mmps': struct.unpack_from('<h', data, 8)[0],
        'velR_mmps': struct.unpack_from('<h', data, 10)[0],
        'v_target_mmps': struct.unpack_from('<h', data, 12)[0],
        'gyroZ_dps10': struct.unpack_from('<h', data, 14)[0],
        'encL_delta': struct.unpack_from('<b', data, 16)[0],
        'encR_delta': struct.unpack_from('<b', data, 17)[0],
        'state': data[18],
    }

def parse_slow(data: bytes) -> dict:
    if not verify_crc(data): return None
    return {
        'type': data[0], 'seq': data[1],
        't_ms':         struct.unpack_from('<H', data, 2)[0],
        'missed_ticks': struct.unpack_from('<H', data, 4)[0],
        'rx_drop':      struct.unpack_from('<H', data, 6)[0],
        'crc_fail':     struct.unpack_from('<H', data, 8)[0],
        'last_cmd_op':  data[10],
        'last_status':  data[11],
        'wd_age_ms':    struct.unpack_from('<H', data, 12)[0],
        'step_max_us':  struct.unpack_from('<H', data, 14)[0],
        'vBat_avg_mV':  struct.unpack_from('<H', data, 16)[0],
        'active_flags': data[18],
    }

# ── RSP payload parsers (test-specific) ──────────────────────────────────────

def parse_rsp_battery(payload: bytes) -> dict:
    """Parse READ_BATTERY RSP payload (5 bytes)"""
    return {
        'vBat_raw_mV': struct.unpack_from('<H', payload, 0)[0],
        'vBat_avg_mV': struct.unpack_from('<H', payload, 2)[0],
        'level': payload[4],
    }

def parse_rsp_sensors(payload: bytes) -> dict:
    """Parse READ_SENSORS RSP payload (9 bytes)"""
    return {
        'left_mm':  struct.unpack_from('<H', payload, 0)[0],
        'fl_mm':    struct.unpack_from('<H', payload, 2)[0],
        'fr_mm':    struct.unpack_from('<H', payload, 4)[0],
        'right_mm': struct.unpack_from('<H', payload, 6)[0],
        'bitmap':   payload[8],
    }

def parse_rsp_imu(payload: bytes) -> dict:
    """Parse READ_IMU RSP payload (12 bytes)"""
    return {
        'gyroX':  struct.unpack_from('<h', payload, 0)[0],
        'gyroY':  struct.unpack_from('<h', payload, 2)[0],
        'gyroZ':  struct.unpack_from('<h', payload, 4)[0],
        'accelX': struct.unpack_from('<h', payload, 6)[0],
        'accelY': struct.unpack_from('<h', payload, 8)[0],
        'accelZ': struct.unpack_from('<h', payload, 10)[0],
    }

def parse_rsp_buttons(payload: bytes) -> dict:
    """Parse READ_BUTTONS RSP payload (11 bytes)"""
    return {
        'start': payload[0],
        'mode':  payload[1],
        'dip':   payload[2],
        'encL':  struct.unpack_from('<i', payload, 3)[0],
        'encR':  struct.unpack_from('<i', payload, 7)[0],
    }

def parse_rsp_cal_gyro(payload: bytes) -> dict:
    """Parse CAL_GYRO RSP (8 bytes):
       gyroZ_off×100(i16) gyroX_off×100(i16) gyroY_off×100(i16) calibrated(u8) status(u8)
       status: 0=started, 1=busy, 2=done (offsets valid only when status==2)"""
    st = payload[7] if len(payload) > 7 else 0
    return {
        'gyro_z_off': struct.unpack_from('<h', payload, 0)[0] / 100.0 if st == 2 else 0,
        'gyro_x_off': struct.unpack_from('<h', payload, 2)[0] / 100.0 if st == 2 else 0,
        'gyro_y_off': struct.unpack_from('<h', payload, 4)[0] / 100.0 if st == 2 else 0,
        'calibrated': bool(payload[6]) if st == 2 else False,
        'status':     st,  # 0=started, 1=busy, 2=done
    }

def parse_rsp_cal_walls(payload: bytes) -> dict:
    """Parse CAL_WALLS RSP (13 bytes):
       raw_L(u16) raw_FL(u16) raw_FR(u16) raw_R(u16) off_L(i8) off_FL(i8) off_FR(i8) off_R(i8) status(u8)
       status: 0=started, 1=busy, 2=done (data valid only when status==2)"""
    st = payload[12] if len(payload) > 12 else 0
    if st == 2:
        return {
            'raw_L':   struct.unpack_from('<H', payload, 0)[0],
            'raw_FL':  struct.unpack_from('<H', payload, 2)[0],
            'raw_FR':  struct.unpack_from('<H', payload, 4)[0],
            'raw_R':   struct.unpack_from('<H', payload, 6)[0],
            'off_L':   struct.unpack_from('<b', payload, 8)[0],
            'off_FL':  struct.unpack_from('<b', payload, 9)[0],
            'off_FR':  struct.unpack_from('<b', payload, 10)[0],
            'off_R':   struct.unpack_from('<b', payload, 11)[0],
            'status':  st,
        }
    return {'status': st, 'raw_L':0,'raw_FL':0,'raw_FR':0,'raw_R':0,
            'off_L':0,'off_FL':0,'off_FR':0,'off_R':0}

def parse_rsp_wall_set_offset(payload: bytes) -> dict:
    """Parse WALL_SET_OFFSET RSP (6 bytes):
       pos(u8) offset(i8) all_offsets(i8×4)"""
    return {
        'pos':    payload[0],
        'offset': struct.unpack_from('<b', payload, 1)[0],
        'off_L':  struct.unpack_from('<b', payload, 2)[0],
        'off_FL': struct.unpack_from('<b', payload, 3)[0],
        'off_FR': struct.unpack_from('<b', payload, 4)[0],
        'off_R':  struct.unpack_from('<b', payload, 5)[0],
    }

def parse_rsp_cal_query(payload: bytes) -> dict:
    """Parse CAL_QUERY RSP (13 bytes)"""
    return {
        'gyro_z_off': struct.unpack_from('<h', payload, 0)[0] / 100.0,
        'calibrated': bool(payload[2]),
        'off_L':      struct.unpack_from('<b', payload, 3)[0],
        'off_FL':     struct.unpack_from('<b', payload, 4)[0],
        'off_FR':     struct.unpack_from('<b', payload, 5)[0],
        'off_R':      struct.unpack_from('<b', payload, 6)[0],
    }

def parse_rsp_walls(payload: bytes) -> dict:
    """Parse READ_WALLS RSP payload (13 bytes):
       L(i16) FL(i16) FR(i16) R(i16) flags(u8) hdg(u8) x(u8) y(u8) debounce(u8)"""
    flags = payload[8]
    dbc = payload[12] if len(payload) >= 13 else 0
    return {
        'left_mm':      struct.unpack_from('<h', payload, 0)[0],
        'front_l_mm':   struct.unpack_from('<h', payload, 2)[0],
        'front_r_mm':   struct.unpack_from('<h', payload, 4)[0],
        'right_mm':     struct.unpack_from('<h', payload, 6)[0],
        'det_left':     bool(flags & 1),
        'det_front_l':  bool(flags & 2),
        'det_front_r':  bool(flags & 4),
        'det_right':    bool(flags & 8),
        'det_front':    bool(flags & 16),   # combined front (per WALL_FRONT_DETECT_MODE)
        'heading':      payload[9],
        'x':            payload[10],
        'y':            payload[11],
        'dbc_l':        dbc & 0x03,
        'dbc_fl':       (dbc >> 2) & 0x03,
        'dbc_fr':       (dbc >> 4) & 0x03,
        'dbc_r':        (dbc >> 6) & 0x03,
    }

def parse_rsp_search_query(payload: bytes) -> dict:
    """Parse SEARCH_QUERY RSP (13 bytes):
       state mode x y heading steps(u16) error walls visited_count
       goal_x_min|max goal_y_min|max start_x|y"""
    b10 = payload[10] if len(payload) > 10 else 0
    b11 = payload[11] if len(payload) > 11 else 0
    b12 = payload[12] if len(payload) > 12 else 0
    return {
        'search_state': payload[0],
        'mode':         payload[1],
        'x':            payload[2],
        'y':            payload[3],
        'heading':      payload[4],
        'steps':        struct.unpack_from('<H', payload, 5)[0],
        'error_code':   payload[7],
        'walls_here':   payload[8],
        'visited_count':payload[9],
        'goal_x_min':   b10 & 0x0F,
        'goal_x_max':   (b10 >> 4) & 0x0F,
        'goal_y_min':   b11 & 0x0F,
        'goal_y_max':   (b11 >> 4) & 0x0F,
        'start_x':      b12 & 0x0F,
        'start_y':      (b12 >> 4) & 0x0F,
    }

def parse_rsp_maze_row(payload: bytes) -> dict:
    """Parse MAZE_ROW RSP (13 bytes):
       row, 8 bytes packed walls (2 cells/byte), 2 bytes visited bitmask"""
    row = payload[0]
    walls = []
    for i in range(8):
        b = payload[1 + i]
        walls.append(b & 0x0F)        # even cell
        walls.append((b >> 4) & 0x0F)  # odd cell
    vis_bits = struct.unpack_from('<H', payload, 9)[0]
    visited = [(vis_bits >> i) & 1 for i in range(16)]
    return {'row': row, 'walls': walls, 'visited': visited}

def parse_rsp_speedrun(payload: bytes) -> dict:
    """Parse SPEEDRUN_QUERY RSP (10 bytes)"""
    return {
        'state':       payload[0],
        'speed_level': payload[1],
        'path_len':    struct.unpack_from('<H', payload, 2)[0],
        'progress':    struct.unpack_from('<H', payload, 4)[0],
        'num_segs':    struct.unpack_from('<H', payload, 6)[0],
        'cur_seg':     struct.unpack_from('<H', payload, 8)[0],
    }

def state_str(s: int) -> str:
    """Human-readable state byte"""
    parts = []
    if s & FLAG_ARMED:    parts.append("ARMED")
    if s & FLAG_RUNNING:  parts.append("RUNNING")
    if s & FLAG_WATCHDOG: parts.append("WD")
    if s & FLAG_LOW_BATT: parts.append("LOW_BAT")
    base = s & 0x0F
    names = {0: "IDLE", 1: "ARMED", 2: "RUNNING", 3: "STOPPING", 4: "ERROR"}
    parts.insert(0, names.get(base, f"?{base}"))
    return "|".join(parts)
