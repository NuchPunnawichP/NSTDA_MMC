// ═══════════════════════════════════════════════════════════════════════════════
//  config.h — Micromouse ESP32-S3 Hardware & Tuning Configuration
//
//  Layer: 0 (Configuration — no dependencies)
//  Target: ESP32-S3, Arduino core 3.3.5
//
//  ★★★ นักเรียน: กรอกค่าที่ได้จากการทดสอบในแต่ละ Lab ★★★
//
//  หน่วยหลัก: mV (millivolt) ทั้งระบบ
//    Motor command = vCmd_mV → HAL คำนวณ duty = vCmd_mV / vBat_mV
//    ห้ามสั่ง PWM โดยตรงจากชั้นบน (Iron Rule)
// ═══════════════════════════════════════════════════════════════════════════════
#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ┌───────────────────────────────────────────────────────────────────────────
// │ SECTION 0: LAB SELECTION (active lab at compile time, overridable by BLE)
// └───────────────────────────────────────────────────────────────────────────
// 1  = Lab 1:  Hardware Check
// 2  = Lab 2a: Encoder CPR (hand-spin test)
// 3  = Lab 2b: Motor Characterization (FF params)
// 4  = Lab 3:  Motion Profile (Trapezoidal / S-curve)
// 5  = Lab 4:  PID Tuning (straight, turn, arc)
// 6  = Lab 5:  Search Run (flood fill)
// 7  = Lab 6:  Fast Run
#define DEFAULT_LAB_ID          1

// ┌───────────────────────────────────────────────────────────────────────────
// │ SECTION 1: PIN DEFINITIONS
// │ (ไม่ต้องแก้ไข ยกเว้นเปลี่ยน hardware layout)
// └───────────────────────────────────────────────────────────────────────────

// ── I2C Bus 0: IMU (MPU6050 / GY-521) ──────────────────────────────────────
#define PIN_I2C0_SDA            8
#define PIN_I2C0_SCL            9
#define I2C0_FREQ               400000  // 400 kHz Fast Mode

// ── I2C Bus 1: Wall Sensors (VL6180X) ───────────────────────────────────────
//    ถ้าใช้ IR analog อย่างเดียว สามารถตั้ง I2C_BUS1_ENABLED = 0
#define I2C_BUS1_ENABLED        1       // 0 = ไม่ใช้ I2C Bus 1
#define PIN_I2C1_SDA            2
#define PIN_I2C1_SCL            1
#define I2C1_FREQ               400000

// ── VL6180X XSHUT Pins (สำหรับ re-addressing) ──────────────────────────────
//    -1 = ไม่ใช้ตำแหน่งนี้
#define PIN_VL_XSHUT_LEFT       -1      // Sensor slot 0: Left
#define PIN_VL_XSHUT_FRONT_L    -1      // Sensor slot 1: Front-Left
#define PIN_VL_XSHUT_FRONT_R    -1      // Sensor slot 2: Front-Right
#define PIN_VL_XSHUT_RIGHT      -1      // Sensor slot 3: Right

// VL6180X I2C addresses (after re-addressing)
#define VL6180X_ADDR_DEFAULT    0x29
#define VL6180X_ADDR_LEFT       0x30
#define VL6180X_ADDR_FRONT_L    0x31
#define VL6180X_ADDR_FRONT_R    0x32
#define VL6180X_ADDR_RIGHT      0x33

// ── IR Analog Sensors (SHARP GP2Y0A51SK0F) ──────────────────────────────────
//    -1 = ไม่ใช้ตำแหน่งนี้
#define PIN_IR_LEFT             11      // ADC2_CH0
#define PIN_IR_FRONT_L          12      // ADC2_CH1
#define PIN_IR_FRONT_R          13      // ADC2_CH2
#define PIN_IR_RIGHT            14      // ADC2_CH3

// ── Motors (DRV8833 Dual H-Bridge) ──────────────────────────────────────────
#define PIN_MOTOR_L_IN1         5
#define PIN_MOTOR_L_IN2         4
#define PIN_MOTOR_R_IN1         6
#define PIN_MOTOR_R_IN2         7

// ── Encoders (N20 Quadrature) ───────────────────────────────────────────────
#define PIN_ENC_L_A             15
#define PIN_ENC_L_B             16
#define PIN_ENC_R_A             17
#define PIN_ENC_R_B             18

// ── Battery ADC ─────────────────────────────────────────────────────────────
#define PIN_BATTERY_ADC         10

// ── User Interface ──────────────────────────────────────────────────────────
#define PIN_RGB_LED             48      // WS2812 onboard LED

#define PIN_BUTTON_START        21      // Start / Stop button
#define PIN_BUTTON_MODE         41      // Mode select button

#define PIN_DIP_SW_0            37      // DIP Switch bit 0 (LSB)
#define PIN_DIP_SW_1            38      // DIP Switch bit 1
#define PIN_DIP_SW_2            39      // DIP Switch bit 2
#define PIN_DIP_SW_3            40      // DIP Switch bit 3 (MSB)

// ┌───────────────────────────────────────────────────────────────────────────
// │ SECTION 2: WALL SENSOR MANAGER
// │ ★ นักเรียน: เลือกประเภทและตำแหน่งเซนเซอร์ตามหุ่นของตัวเอง ★
// │
// │ Sensor Manager จัดการทุกเซนเซอร์ผ่าน abstraction layer เดียว
// │ ชั้นบนเห็นแค่ "wall_mm[position]" ไม่ต้องรู้ว่าเป็น IR หรือ VL6180X
// └───────────────────────────────────────────────────────────────────────────

// เลือก 1 = IR analog, 2 = VL6180X (I2C ToF), 3 = Mixed
// (ต้องตรงกับ WALL_SNS_* ใน protocol.h)
#define WALL_SENSOR_TYPE        1       // 1=IR, 2=VL6180X, 3=Mixed

// จำนวนเซนเซอร์ที่ใช้ (3–6 ตำแหน่ง, แนะนำ 4)
#define WALL_SENSOR_COUNT       4

// การกำหนดประเภทต่อตำแหน่ง (ใช้เมื่อ WALL_SENSOR_TYPE = 3 Mixed)
//   0 = ไม่ใช้, 1 = IR analog, 2 = VL6180X
#define SLOT_TYPE_LEFT          1
#define SLOT_TYPE_FRONT_L       1
#define SLOT_TYPE_FRONT_R       1
#define SLOT_TYPE_RIGHT         1

// ── Wall detection thresholds (mm) — per sensor ← ทดสอบใน Lab 1 ──────────
//   วิธี calibrate: วางหุ่นในช่อง → Lab1 READ_SENSORS หรือ Lab5 READ_WALLS
//   จดค่า "มีกำแพง" vs "ไม่มี" → ตั้ง threshold ตรงกลาง
//
//   ★ Best practice สำหรับ front sensors ที่เอียง 45°:
//     • IR ที่ 45° อ่านระยะกำแพงได้ ~1.41× ระยะจริง (เพราะมุมตกกระทบ)
//     • ถ้าระยะจริง 50mm → sensor อ่านได้ ~71mm
//     • แนะนำ: ตั้ง front threshold สูงกว่า side 20-30% เพื่อ compensate
//     • OR mode (default) ปลอดภัยสุดเพราะจับได้แม้เห็นแค่ด้านเดียว
//     • AND mode ดีเวลาต้องการหลบ false positive จากเสากลางเขาวงกต
//
//   ★ Hysteresis: ป้องกัน wall state กระพริบที่ขอบ threshold
//     • ON  = ระยะที่ "ตรวจจับ" กำแพง  (distance < ON → wall = true)
//     • OFF = ระยะที่ "ปล่อย" กำแพง     (distance > OFF → wall = false)
//     • เมื่อ ON < dist < OFF → คง state เดิม (dead zone)
//     • ★ ต้องตั้ง OFF > ON เสมอ
//
//   ★ Debounce: จำนวนครั้งที่ต้องอ่านค่าตรงกันก่อนเปลี่ยน state
//     • 3 = 3ms (ที่ 1kHz) — recommended: ตอบสนองเร็ว แต่กรอง noise
//     • 5 = 5ms — เสถียรกว่าแต่หน่วงเล็กน้อย
//     • ที่ 200mm/s → 3ms ≈ 0.6mm movement (ไม่กระทบ accuracy)
//
//  ┌─────────────────────────────────────────────────────────────┐
//  │  Sensor      │  ON (mm) │  OFF (mm) │  Dead zone (mm)      │
//  ├─────────────────────────────────────────────────────────────┤
//  │  Front-L 45° │    55    │    70     │  55-70 (15mm gap)    │
//  │  Front-R 45° │    55    │    70     │  55-70 (15mm gap)    │
//  │  Side-L  90° │    75    │    90     │  75-90 (15mm gap)    │
//  │  Side-R  90° │    75    │    90     │  75-90 (15mm gap)    │
//  └─────────────────────────────────────────────────────────────┘

// ── Hysteresis thresholds per sensor ──
#define WALL_HYST_FL_ON_MM       0     // Front-Left: detect wall (distance < ON)
#define WALL_HYST_FL_OFF_MM      0     // Front-Left: release wall (distance > OFF)
#define WALL_HYST_FR_ON_MM       0     // Front-Right: detect wall
#define WALL_HYST_FR_OFF_MM      0     // Front-Right: release wall
#define WALL_HYST_SL_ON_MM       0     // Side-Left: detect wall  (★ 65: margin 12mm from 53mm hugging)
#define WALL_HYST_SL_OFF_MM      0     // Side-Left: release wall (★ 80: clean gap detect vs >100mm open)
#define WALL_HYST_SR_ON_MM       0     // Side-Right: detect wall
#define WALL_HYST_SR_OFF_MM      0     // Side-Right: release wall

// ── Debounce (consecutive samples at 1kHz to change state) ──
#define WALL_DEBOUNCE_COUNT      3      // 3ms at 1kHz (default, recommended)

// ── Legacy simple thresholds (backward compat, used as midpoint) ──
#define WALL_THRESH_FRONT_L_MM   WALL_HYST_FL_ON_MM
#define WALL_THRESH_FRONT_R_MM   WALL_HYST_FR_ON_MM
#define WALL_THRESH_SIDE_L_MM    WALL_HYST_SL_ON_MM
#define WALL_THRESH_SIDE_R_MM    WALL_HYST_SR_ON_MM

// Front wall detection mode:
//   0 = OR   → either FL or FR detected  (safest ★แนะนำ — ป้อง missed wall)
//   1 = AND  → both FL and FR detected   (ป้อง false positive จากเสา/มุม)
//   2 = AVG  → (FL+FR)/2 below avg threshold  (smooth, good for symmetric setup)
//   3 = MIN  → min(FL,FR) below lower threshold (เชื่อ sensor ใกล้กว่า)
//   4 = FL_ONLY → use front-left sensor only
//   5 = FR_ONLY → use front-right sensor only
//
//   ★ Best practice กับ front sensors 45°:
//     • Mode 0 (OR):  default ดีสำหรับเริ่มต้น จับกำแพงได้ทุกทิศ
//     • Mode 1 (AND): ใช้เมื่อ tuned แล้ว ลด false positive ที่เกิดจาก
//                      sensor ตัวเดียวเห็นเสาเขาวงกตที่มุม cell
//     • Mode 3 (MIN): ดีสำหรับ continuous mode — ใช้ sensor ที่เห็นกำแพงชัดกว่า
//     • ถ้าหุ่นเบี้ยว/จัดแนวไม่ดี ให้ใช้ Mode 0 ไปก่อน
#define WALL_FRONT_DETECT_MODE   0

// Legacy aliases (backward compat)
#define WALL_THRESHOLD_FRONT_MM  ((WALL_HYST_FL_ON_MM + WALL_HYST_FR_ON_MM) / 2)
#define WALL_THRESHOLD_SIDE_MM   ((WALL_HYST_SL_ON_MM + WALL_HYST_SR_ON_MM) / 2)

// ── Arc Turn: Side sensor gap detection (ตรวจช่องเปิดข้างทาง) ──
//   เมื่อ side sensor "เห็นกำแพงหาย" ระหว่างวิ่ง → อาจเริ่ม arc turn ล่วงหน้า
//   ★ ใช้กับ continuous mode + arc turns (Lab 6 advanced)
//
//   Gap detection logic:
//     1. ระหว่างวิ่งตรง: sample side sensor ที่ 1kHz
//     2. เมื่อ side sensor เปลี่ยนจาก "เห็นกำแพง" → "ไม่เห็น" = gap leading edge
//     3. รอ debounce + verify ด้วย consecutive samples
//     4. เมื่อ confirm gap → คำนวณ arc turn entry point
//
//   Lookahead: ระยะที่ต้อง "เห็นช่อง" ก่อนถึง cell center ถึงจะเริ่ม arc
//     • ค่าน้อย = เลี้ยวช้า (safe) / ค่ามาก = เลี้ยวเร็ว (aggressive)
//     • ★ ต้อง tune ให้เข้ากับ turn radius + velocity
#define ARC_TURN_ENABLE          0      // 0=disabled, 1=enabled (Lab 6 advanced)
#define ARC_GAP_LOOKAHEAD_MM     0     // ★ 50mm: match sensor gap detection @200mm/s
#define ARC_GAP_DEBOUNCE         2      // ★ 2: 67ms@30Hz latency (5→167ms = miss trigger!)
#define ARC_TURN_RADIUS_MM       0     // ★ 40mm: match 50mm lookahead timing window

// Wall sensor offsets (mm, ค่าลบ = อ่านเกิน, บวก = อ่านขาด) ← Lab 1
#define WALL_OFFSET_LEFT_MM     0
#define WALL_OFFSET_FRONT_L_MM  0
#define WALL_OFFSET_FRONT_R_MM  0
#define WALL_OFFSET_RIGHT_MM    0

// VL6180X valid range
#define VL6180X_RANGE_MIN_MM    5
#define VL6180X_RANGE_MAX_MM    200

// IR sensor: ADC → mm conversion (ปรับตาม datasheet / calibration curve)
// GP2Y0A51SK0F: ~2-15 cm range, output = inverse relationship
// ค่านี้ student ต้อง calibrate เอง
#define IR_ADC_BITS             12      // ESP32 ADC = 12-bit
#define IR_MIN_VALID_MM         10
#define IR_MAX_VALID_MM         150

// ┌───────────────────────────────────────────────────────────────────────────
// │ SECTION 3: MOTOR PARAMETERS
// │ ★ Lab 1: ตรวจสอบทิศทาง ★
// └───────────────────────────────────────────────────────────────────────────

#define MOTOR_PWM_FREQ          20000   // 20 kHz (above audible range)
#define MOTOR_PWM_RESOLUTION    10      // 10-bit (0–1023)
#define MOTOR_PWM_MAX           1023    // max duty value (internal HAL use only)

// Direction compensation (1 = ปกติ, -1 = กลับทิศ) ← ทดสอบใน Lab 1
#define MOTOR_L_DIRECTION       1
#define MOTOR_R_DIRECTION       1

// Encoder direction (1 = ปกติ, -1 = กลับทิศ) ← ทดสอบใน Lab 1
#define ENC_L_DIRECTION         1
#define ENC_R_DIRECTION         1

// ┌───────────────────────────────────────────────────────────────────────────
// │ SECTION 4: ROBOT PHYSICAL PARAMETERS
// │ ★ Lab 2a: วัดค่า Encoder CPR (counts per revolution) ★
// └───────────────────────────────────────────────────────────────────────────

#define WHEEL_DIAMETER_MM       0.0f   // เส้นผ่านศูนย์กลางล้อ (mm)
#define WHEEL_BASE_MM           0.0f   // ระยะห่างระหว่างล้อ center-to-center (mm)
#define COUNTS_PER_REV          0.0f  // Encoder CPR (x4 quadrature) ← Lab 2a
#define GEAR_RATIO              1.0f    // Gear ratio (ถ้ามี gearbox)

// Derived (อย่าแก้)
#define WHEEL_CIRCUMFERENCE_MM  (WHEEL_DIAMETER_MM * 3.14159265f)
#define MM_PER_COUNT            (WHEEL_CIRCUMFERENCE_MM / COUNTS_PER_REV)

// ┌───────────────────────────────────────────────────────────────────────────
// │ SECTION 5: IMU CALIBRATION (MPU6050 / GY-521)
// │ ★ Lab 1: วัดค่า offset เมื่อหุ่นอยู่นิ่ง ★
// └───────────────────────────────────────────────────────────────────────────

#define IMU_I2C_ADDR            0x68    // MPU6050 default

// Gyroscope raw offset (วัดเมื่อหุ่นนิ่ง ← Lab 1)
#define GYRO_OFFSET_X           0.0f
#define GYRO_OFFSET_Y           0.0f
#define GYRO_OFFSET_Z           0.0f

// Gyro Z sign convention for heading:
//   Robot CW turn (มองจากด้านบน) = positive heading change
//   MPU6050: CW = negative gyroZ (right-hand rule) → ต้อง flip sign
//   ★ ถ้าหมุน CW แล้ว heading ไปทางลบ → เปลี่ยนเป็น +1
//   ★ ถ้าหมุน CW แล้ว heading ไปทางบวก → เปลี่ยนเป็น -1
#define GYRO_Z_SIGN             (-1)    // -1 = flip (MPU6050 default)

// Accelerometer raw offset (No used anymore, Just Fallback) ไม่ต้องใช้เรามีฟังก์ชัน Auto Cal ตอนเริ่มไฟสี Cyan
#define ACCEL_OFFSET_X          0.0f
#define ACCEL_OFFSET_Y          0.0f
#define ACCEL_OFFSET_Z          0.0f

// Gyroscope sensitivity (depends on range setting)
#define GYRO_RANGE_DPS          500     // ±500 °/s
#define GYRO_SCALE_DPS          (500.0f / 32768.0f)

// ┌───────────────────────────────────────────────────────────────────────────
// │ SECTION 6: FEEDFORWARD PARAMETERS (Peter Harrison Method)
// │ ★ Lab 2b: Motor Characterization ★
// │
// │ สูตร: vCmd_mV = bias_mV + speed_ff × velocity + acc_ff × acceleration
// │ หน่วย: ทุกอย่างเป็น mV (ห้ามใช้ PWM ในชั้นนี้)
// │
// │ วิธีหาค่า:
// │   1. bias_mV: แรงดันต่ำสุดที่ล้อเริ่มหมุน (static friction)
// │   2. speed_ff: slope ของกราฟ vCmd vs velocity (mV per mm/s)
// │   3. acc_ff: speed_ff × τ (time constant from step response)
// │   4. tau_ms: เวลาที่ velocity ถึง 63% ของ steady-state
// └───────────────────────────────────────────────────────────────────────────

// ── Linear Motion (Forward/Backward) ────────────────────────────────────────
// ★ Lab 2b MEASURED VALUES ★
// Ramp sweep 2000→7000mV: Bias L≈4500, R≈4600, @7000mV vel≈800mm/s
// Step response 5000mV: vel≈320mm/s
// FF = (Cmd - Bias) / Vel ≈ 3.1 mV/(mm/s)
// ── Linear feedforward (per-wheel) ─────────────────────────────────────────────
// kS (bias) from Lab2b FIND_BIAS (median of 10):
#define FF_LINEAR_BIAS_L_MV     0    // Left motor stiction bias (mV)
#define FF_LINEAR_BIAS_R_MV     0    // Right motor stiction bias (mV)
#define FF_LINEAR_BIAS_MV       0    // Average (used only by ff_voltage())
#define FF_LINEAR_SPEED_L_X100  0     // mV/(mm/s) × 100 = 3.44 mV per mm/s
#define FF_LINEAR_SPEED_R_X100  0     // mV/(mm/s) × 100 = 3.24 mV per mm/s
#define FF_LINEAR_ACC_X100      0      // mV/(mm/s²) × 100 ← tune in Lab 4
#define FF_LINEAR_TAU_MS        0     // motor time constant (ms) ← tune later

// ── Rotation (Turn in place) ────────────────────────────────────────────────
#define FF_ROT_BIAS_MV          0    // mV to start rotation (avg of L/R)
#define FF_ROT_SPEED_X100       0     // mV/(°/s) × 100 ← tune in Lab 4
#define FF_ROT_ACC_X100         0      // mV/(°/s²) × 100 ← tune in Lab 4
#define FF_ROT_TAU_MS           0     // ms ← tune later

// Velocity estimation window (ms) used by ControlTask (Lab4 PID feedback)
// 1 = raw 1ms delta (very quantized). Recommended: 5–10.
#define VEL_EST_WINDOW_MS       10
// IMPORTANT: run-bias (dynamic intercept) from your Lab2b SS-Vel regression
// This fixes the "vel=200 but runs ~300" symptom.
#define FF_LINEAR_RUN_BIAS_L_MV   0
#define FF_LINEAR_RUN_BIAS_R_MV   0
// Blend zone for start-boost (near 0 speed use kS, at higher speed use run-bias)
#define FF_START_BLEND_V0_MMPS    0.0f
#define FF_START_BLEND_V1_MMPS    0.0f

// ┌───────────────────────────────────────────────────────────────────────────
// │ SECTION 7: PD CONTROLLER PARAMETERS
// │ ★ Lab 4: PID Tuning ★
// │
// │ Control: Total = Feedforward + PD correction
// │ PD output is in mV (added to FF output)
// │ Kp, Kd stored as ×100 integers internally
// └───────────────────────────────────────────────────────────────────────────

// ── Forward/Backward PD ─────────────────────────────────────────────────────
#define PD_LINEAR_KP_X100       0     // Kp = 5.00 mV per mm/s error ← Lab 4
#define PD_LINEAR_KD_X100       0       // Kd = 0 (disabled) ← Lab 4
#define PD_LINEAR_MAX_MV        0     // max correction (mV)

// ── Rotation/Heading PD ─────────────────────────────────────────────────────
#define PD_ROT_KP_X100          0     // Kp = 8.00 mV per ° error ← Lab 4
#define PD_ROT_KD_X100          0       // Kd = 0 (disabled) ← Lab 4
#define PD_ROT_MAX_MV           0

// ── Steering PD (keep straight while forward) ───────────────────────────────
#define PD_STEER_KP_X100        0    // ← Lab 4
#define PD_STEER_KI_X100        0      // Ki = 0.10  ← แก้ constant drift (เอียงซ้าย/ขวา)
#define PD_STEER_KD_X100        0      // ← Lab 4
#define PD_STEER_MAX_MV         0

// ── Steering Trim (constant mV offset) ──────────────────────────────────────
// บวก = แก้เอียงซ้าย (เพิ่มแรงล้อซ้าย), ลบ = แก้เอียงขวา
// ★ ถ้าเอียงซ้ายตลอด → เพิ่มค่าเป็น +50~+200
// ★ ปรับผ่าน GUI live ได้ → จดค่าที่ดีกลับมาใส่ที่นี่
#define FF_STEERING_TRIM_MV     0       // ← Lab 4 (ปรับจาก GUI)

// ── Wall Follow PD ──────────────────────────────────────────────────────────
#define PD_WALL_KP_X100         0     // ← Lab 4
#define PD_WALL_KD_X100         0      // ← Lab 4
#define PD_WALL_MAX_MV          0

// ── Wall Centering (Lab 5 search / wall follow) ─────────────────────────────
//  Used during straight driving to keep robot centered in the passage.
//  Correction is applied as steering offset (same direction as heading PID).
#define WALL_CENTER_TARGET_MM   0      // ideal side-wall distance when centered
#define WALL_CENTER_BOTH_GAIN   0.0f    // when both walls: error = (L-R)*gain
#define WALL_CENTER_ONE_GAIN    0.0f    // when one wall: error = (measured-target)*gain
#define WALL_CENTER_ENABLE      1       // 1=enable wall centering, 0=gyro only
// Wall follow test (standalone left/right wall follow for testing)
#define WALL_FOLLOW_VEL_MMPS    0     // mm/s for wall follow test
#define WALL_FOLLOW_FRONT_STOP_MM 0    // stop if front wall closer than this

// ┌───────────────────────────────────────────────────────────────────────────
// │ SECTION 8: MOTION PARAMETERS
// │ ★ Lab 3: Motion Profile / Lab 4: PID Tuning ★
// └───────────────────────────────────────────────────────────────────────────

// Maze cell size (competition standard)
#define CELL_SIZE_MM            180.0f

// Search run (slow, safe) ← Lab 5
#define SEARCH_VEL_MMPS         200     // mm/s (safe for 30Hz sensor: 6.6mm/update)
#define SEARCH_ACCEL_MMPS2      2000    // mm/s² (★ smooth accel)
#define SEARCH_DECEL_MMPS2      2500    // mm/s² (★ firm decel: 8mm stopping distance)

// ── Front Safety System (Lab 5) ─────────────────────────────────────────────
//
//  Geometry: front sensor offset = 64mm ahead of robot center
//    → sensor reads 26mm when robot at cell center, wall at boundary
//    → sensor reads ~100mm when robot enters cell (center at boundary)
//    → sensor reads 150mm (IR max) when wall is >206mm away
//
//  Three protection layers:
//    Layer 1: Emergency Stop   — hard stop if critically close (< 15mm)
//    Layer 2: Progressive Brake — gradual speed reduction (80→35mm)
//    Layer 3: Post-turn Recheck — re-read front after turning, before driving
//
//  Brake profile @ 200mm/s with wall at destination boundary:
//    sensor 80mm → travel 126mm (70%) → start brake
//    sensor 50mm → travel 156mm (87%) → heavy brake
//    sensor 35mm → travel 171mm (95%) → creep 50mm/s
//    sensor 26mm → travel 180mm        → arrived at center

// Layer 1: Emergency Stop (always on, catches worst case)
#define FRONT_ESTOP_ENABLE       1
#define FRONT_ESTOP_MM           15      // raw mm threshold (IR reliable ≥10mm)
#define FRONT_ESTOP_COUNT        3       // ticks @1kHz = 3ms (instant vs 30Hz sensor)
//  worst case: creep 50mm/s → confirm 0.15mm + brake 0.5mm = 0.65mm << 15mm ✔
//  no-brake:   200mm/s → confirm 0.6mm + brake 8mm = 8.6mm < 15mm ✔

// Layer 2: Progressive Brake (gradual deceleration)
#define FRONT_BRAKE_ENABLE       1
#define FRONT_BRAKE_TH1_MM       80      // full→50%  (travel ~126mm, 70% of cell)
#define FRONT_BRAKE_TH2_MM       50      // 50%→25%   (travel ~156mm, 87% of cell)
#define FRONT_BRAKE_TH3_MM       35      // creep zone (travel ~171mm, 95% of cell)
#define FRONT_BRAKE_CREEP_MMPS   50      // creep speed (braking dist 0.5mm)

// Layer 3: Post-turn Recheck (detect wall after turning)
#define FRONT_RECHECK_ENABLE     1
#define FRONT_RECHECK_MM         40      // wall threshold: 26mm(wall) vs 100mm(open)

// Fast run (high speed) ← Lab 6
#define FAST_VEL_MMPS           500     // mm/s
#define FAST_ACCEL_MMPS2        1000    // mm/s²
#define FAST_DECEL_MMPS2        1000    // mm/s²

// Lab 6 speed levels (0=cautious → 3=fastest)
//   Level 0: same as search speed (safe test)
//   Level 1: moderate — 350mm/s
//   Level 2: fast — 500mm/s
//   Level 3: fastest — 700mm/s (competition)
#define SPEEDRUN_VEL_0_MMPS     200
#define SPEEDRUN_VEL_1_MMPS     350
#define SPEEDRUN_VEL_2_MMPS     500
#define SPEEDRUN_VEL_3_MMPS     700
#define SPEEDRUN_ACCEL_MMPS2    1000
#define SPEEDRUN_DECEL_MMPS2    1000
#define SPEEDRUN_TURN_DPS       300     // faster turns for speed run

// Turn parameters ← Lab 4
#define TURN_VEL_MMPS           100     // mm/s during turn
#define TURN_VEL_DPS            200     // °/s during turn
#define TURN_ACCEL_DPS2         500     // °/s²

// ┌───────────────────────────────────────────────────────────────────────────
// │ SECTION 9: BATTERY MONITORING
// └───────────────────────────────────────────────────────────────────────────

// Voltage divider: Rtop=100kΩ, Rbottom=33kΩ
// ★ Calibrated ratio: วัดจริงด้วย multimeter แล้วคำนวณ
//   ratio = V_bat_multimeter / V_adc_pin
//   ถ้าค่ายังไม่ตรง ให้ปรับตัวเลขนี้จนค่าตรง multimeter
#define BATTERY_DIVIDER_RATIO   4.14f
#define BATTERY_DIVIDER_X100    414     // integer version for hal_battery

// ESP32-S3 ADC
#define ADC_REFERENCE_MV        3300    // 3.3V reference
#define ADC_RESOLUTION_BITS     12
#define ADC_MAX_VALUE           4095    // 2^12 - 1

// 2S LiPo voltage thresholds (mV)
#define BATT_FULL_MV            8400    // 4.2V × 2
#define BATT_NOMINAL_MV         7400    // 3.7V × 2
#define BATT_LOW_MV             7000    // 3.5V × 2 (LED yellow)
#define BATT_CRITICAL_MV        6400    // 3.2V × 2 (LED red, stop motors)

// Battery LED color thresholds
//   Green : vBat >= BATT_LOW_MV
//   Yellow: BATT_CRITICAL_MV <= vBat < BATT_LOW_MV
//   Red   : vBat < BATT_CRITICAL_MV

// Maximum motor command voltage clamp (mV)
// Prevents commanding more than battery can provide
#define VCMD_MAX_MV             8000

// ┌───────────────────────────────────────────────────────────────────────────
// │ SECTION 10: FreeRTOS / SYSTEM PARAMETERS
// │ (ไม่ต้องแก้ไข ยกเว้นต้องการปรับ performance)
// └───────────────────────────────────────────────────────────────────────────

// Control loop timing (Iron Rule: esp_timer + TaskNotify, 1kHz)
#define CONTROL_FREQ_HZ         1000
#define CONTROL_PERIOD_US       1000    // 1 ms

// Telemetry defaults (overridable by BLE CMD)
#define DEFAULT_FAST_HZ         50      // FAST packet rate
#define DEFAULT_SLOW_HZ         5       // SLOW packet rate
#define DEFAULT_STREAM_MASK     0x03    // FAST + SLOW enabled

// Watchdog (BLE disconnect safety)
#define DEFAULT_WATCHDOG_MS     10000   // 10s idle → auto-disarm (GUI sends PING keepalive)

// Task priorities (higher = more important)
//   ControlTask ต้องสูงสุดแต่ไม่สูงจนกัน esp_timer task
#define PRIO_CONTROL            20      // Core1: real-time control
#define PRIO_SENSOR             8       // Core1: sensor reading
#define PRIO_COMMS              5       // Core0: BLE comms
#define PRIO_UI                 2       // Core0: LED + button

// Task stack sizes (bytes)
#define STACK_CONTROL           4096
#define STACK_COMMS             4096
#define STACK_SENSOR            4096
#define STACK_UI                2048

// Core assignments (Iron Rule: Core1 = Control, Core0 = Comms/BLE)
#define CORE_CONTROL            1
#define CORE_SENSOR             1
#define CORE_COMMS              0
#define CORE_UI                 0

// Command mailbox queue depth
#define CMD_QUEUE_DEPTH         8
#define RSP_QUEUE_DEPTH         8

// ┌───────────────────────────────────────────────────────────────────────────
// │ SECTION 11: MAZE PARAMETERS
// └───────────────────────────────────────────────────────────────────────────

#define MAZE_SIZE               16      // 16×16 cells

// Start position (lower-left corner)
#define START_X                 0
#define START_Y                 0
#define START_HEADING_DEG       90      // facing North (+Y)

// Goal position (center of maze)
#define GOAL_X_MIN              7
#define GOAL_Y_MIN              7
#define GOAL_X_MAX              8
#define GOAL_Y_MAX              8

// ┌───────────────────────────────────────────────────────────────────────────
// │ SECTION 12: DIP SWITCH MODES
// │ (4-pin DIP switch = 16 modes)
// └───────────────────────────────────────────────────────────────────────────

typedef enum {
    DIPMODE_IDLE            = 0,    // Wait for BLE command
    DIPMODE_LAB1_HW_CHECK   = 1,    // Hardware check mode
    DIPMODE_LAB2A_ENC       = 2,    // Encoder CPR test
    DIPMODE_LAB2B_MOTOR     = 3,    // Motor characterization
    DIPMODE_LAB3_PROFILE    = 4,    // Motion profile test
    DIPMODE_LAB4_PID        = 5,    // PID tuning
    DIPMODE_LAB5_SEARCH     = 6,    // Search run
    DIPMODE_LAB6_FAST       = 7,    // Fast run
    DIPMODE_CAL_IMU         = 8,    // IMU calibration
    DIPMODE_CAL_WALLS       = 9,    // Wall sensor calibration
    DIPMODE_TEST_FWD        = 10,   // Quick forward test
    DIPMODE_TEST_TURN       = 11,   // Quick turn test
    DIPMODE_RESERVED_12     = 12,
    DIPMODE_RESERVED_13     = 13,
    DIPMODE_RESERVED_14     = 14,
    DIPMODE_RESERVED_15     = 15,
} DipMode_t;

// ┌───────────────────────────────────────────────────────────────────────────
// │ SECTION 13: BLE CONFIGURATION
// └───────────────────────────────────────────────────────────────────────────

// Device name prefix (จะต่อท้ายด้วย _XXXX จาก MAC address)
#define BLE_DEVICE_NAME_PREFIX  "MM"

// BLE connection parameters
#define BLE_MTU_REQUEST         23      // default MTU → payload 20 bytes

// ┌───────────────────────────────────────────────────────────────────────────
// │ SECTION 14: UTILITY MACROS
// └───────────────────────────────────────────────────────────────────────────

#ifndef CONSTRAIN
#define CONSTRAIN(x, lo, hi)    ((x) < (lo) ? (lo) : ((x) > (hi) ? (hi) : (x)))
#endif

#ifndef ABS
#define ABS(x)                  ((x) < 0 ? -(x) : (x))
#endif

#ifndef SIGN
#define SIGN(x)                 ((x) > 0 ? 1 : ((x) < 0 ? -1 : 0))
#endif

#define DEG_TO_RAD(d)           ((d) * 3.14159265f / 180.0f)
#define RAD_TO_DEG(r)           ((r) * 180.0f / 3.14159265f)

// Convert config ×100 value to float (for FF/PD gains)
#define X100_TO_FLOAT(v)        ((float)(v) / 100.0f)

#endif // CONFIG_H
