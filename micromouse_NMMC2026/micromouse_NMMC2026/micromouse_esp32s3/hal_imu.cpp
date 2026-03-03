// ═══════════════════════════════════════════════════════════════════════════════
//  hal_imu.cpp — MPU6050 IMU Implementation (direct Wire access)
//
//  Register map (subset):
//    0x75 WHO_AM_I    = 0x68
//    0x6B PWR_MGMT_1  (write 0x01 = clock from Gyro X PLL)
//    0x1B GYRO_CONFIG (write 0x00 = ±250°/s)
//    0x1C ACCEL_CONFIG(write 0x00 = ±2g)
//    0x3B ACCEL_XOUT_H.. 14 bytes burst (accel XYZ, temp, gyro XYZ)
// ═══════════════════════════════════════════════════════════════════════════════
#include "hal_imu.h"
#include "config.h"
#include <Arduino.h>
#include <Wire.h>

// ── MPU6050 Register Addresses ──────────────────────────────────────────────
#define MPU_REG_WHO_AM_I        0x75
#define MPU_REG_PWR_MGMT_1      0x6B
#define MPU_REG_PWR_MGMT_2      0x6C
#define MPU_REG_GYRO_CONFIG     0x1B
#define MPU_REG_ACCEL_CONFIG    0x1C
#define MPU_REG_CONFIG          0x1A
#define MPU_REG_SMPLRT_DIV      0x19
#define MPU_REG_ACCEL_XOUT_H    0x3B    // start of 14-byte burst

// ── Scaling ─────────────────────────────────────────────────────────────────
#define ACCEL_SENSITIVITY       16384.0f // LSB/g for ±2g range
#define TEMP_SENSITIVITY        340.0f
#define TEMP_OFFSET             36.53f

static float s_gyro_sensitivity = 131.0f;

static uint8_t gyro_cfg_from_range(int dps){
  switch(dps){
    case 250:  s_gyro_sensitivity = 131.0f; return 0x00; // FS_SEL=0
    case 500:  s_gyro_sensitivity = 65.5f;  return 0x08; // FS_SEL=1
    case 1000: s_gyro_sensitivity = 32.8f;  return 0x10; // FS_SEL=2
    case 2000: s_gyro_sensitivity = 16.4f;  return 0x18; // FS_SEL=3
    default:   s_gyro_sensitivity = 65.5f;  return 0x08; // default 500
  }
}

// ── State ───────────────────────────────────────────────────────────────────
static bool       s_init = false;
static ImuRawData s_raw  = {};
static ImuData    s_data = {};

// Calibration offsets (runtime, from hal_imu_calibrate)
static float s_gyro_off_x = 0;
static float s_gyro_off_y = 0;
static float s_gyro_off_z = 0;
static float s_accel_off_x = 0;
static float s_accel_off_y = 0;
static float s_accel_off_z = 0;
static bool  s_calibrated = false;

// ── I2C helpers ─────────────────────────────────────────────────────────────
static void imu_write_reg(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(IMU_I2C_ADDR);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}

static uint8_t imu_read_reg(uint8_t reg) {
    Wire.beginTransmission(IMU_I2C_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);  // repeated start
    Wire.requestFrom((uint8_t)IMU_I2C_ADDR, (uint8_t)1);
    return Wire.available() ? Wire.read() : 0;
}

// ── Public API ──────────────────────────────────────────────────────────────

bool hal_imu_init(void) {
    // Start I2C Bus 0
    Wire.begin(PIN_I2C0_SDA, PIN_I2C0_SCL, I2C0_FREQ);

    // Verify device
    uint8_t id = imu_read_reg(MPU_REG_WHO_AM_I);
    if (id != 0x68 && id != 0x72) {
        // WHO_AM_I mismatch — MPU6050 not found
        s_init = false;
        return false;
    }

    // Wake up (clear SLEEP bit), use Gyro X PLL as clock
    imu_write_reg(MPU_REG_PWR_MGMT_1, 0x01);
    delay(10);

    // Gyro range: ±250°/s (highest sensitivity)
    imu_write_reg(MPU_REG_GYRO_CONFIG, gyro_cfg_from_range(GYRO_RANGE_DPS));

    // Accel range: ±2g (highest sensitivity)
    imu_write_reg(MPU_REG_ACCEL_CONFIG, 0x00);

    // DLPF: bandwidth ~44Hz, delay ~4.9ms (good balance for micromouse)
    imu_write_reg(MPU_REG_CONFIG, 0x03);

    // Sample rate: 1 kHz / (1 + divider) = 1 kHz (divider = 0)
    imu_write_reg(MPU_REG_SMPLRT_DIV, 0x00);

    // Load compile-time offsets as initial calibration
    s_gyro_off_x  = GYRO_OFFSET_X / s_gyro_sensitivity;
    s_gyro_off_y  = GYRO_OFFSET_Y / s_gyro_sensitivity;
    s_gyro_off_z  = GYRO_OFFSET_Z / s_gyro_sensitivity;
    s_accel_off_x = ACCEL_OFFSET_X / ACCEL_SENSITIVITY;
    s_accel_off_y = ACCEL_OFFSET_Y / ACCEL_SENSITIVITY;
    s_accel_off_z = ACCEL_OFFSET_Z / ACCEL_SENSITIVITY;

    delay(50);  // let sensor stabilize
    s_init = true;
    return true;
}

bool hal_imu_update(void) {
    if (!s_init) return false;

    // Burst read 14 bytes: AccelXYZ(6) + Temp(2) + GyroXYZ(6)
    Wire.beginTransmission(IMU_I2C_ADDR);
    Wire.write(MPU_REG_ACCEL_XOUT_H);
    if (Wire.endTransmission(false) != 0) return false;

    uint8_t n = Wire.requestFrom((uint8_t)IMU_I2C_ADDR, (uint8_t)14);
    if (n < 14) return false;

    uint8_t buf[14];
    for (int i = 0; i < 14; i++) buf[i] = Wire.read();

    // Parse big-endian 16-bit values
    s_raw.accelX  = (int16_t)((buf[0]  << 8) | buf[1]);
    s_raw.accelY  = (int16_t)((buf[2]  << 8) | buf[3]);
    s_raw.accelZ  = (int16_t)((buf[4]  << 8) | buf[5]);
    s_raw.temp_raw = (int16_t)((buf[6]  << 8) | buf[7]);
    s_raw.gyroX   = (int16_t)((buf[8]  << 8) | buf[9]);
    s_raw.gyroY   = (int16_t)((buf[10] << 8) | buf[11]);
    s_raw.gyroZ   = (int16_t)((buf[12] << 8) | buf[13]);

    // Convert to physical units
    float gx = (float)s_raw.gyroX / s_gyro_sensitivity;
    float gy = (float)s_raw.gyroY / s_gyro_sensitivity;
    float gz = (float)s_raw.gyroZ / s_gyro_sensitivity;

    // Apply calibration offset
    if (s_calibrated) {
        s_data.gyroX_dps = gx - s_gyro_off_x;
        s_data.gyroY_dps = gy - s_gyro_off_y;
        s_data.gyroZ_dps = gz - s_gyro_off_z;
    } else {
        // Use compile-time offsets (from config.h, pre-divided)
        s_data.gyroX_dps = gx - s_gyro_off_x;
        s_data.gyroY_dps = gy - s_gyro_off_y;
        s_data.gyroZ_dps = gz - s_gyro_off_z;
    }

    float ax = (float)s_raw.accelX / ACCEL_SENSITIVITY;
    float ay = (float)s_raw.accelY / ACCEL_SENSITIVITY;
    float az = (float)s_raw.accelZ / ACCEL_SENSITIVITY;
    s_data.accelX_g = ax - s_accel_off_x;
    s_data.accelY_g = ay - s_accel_off_y;
    s_data.accelZ_g = az - s_accel_off_z;

    s_data.temp_C = (float)s_raw.temp_raw / TEMP_SENSITIVITY + TEMP_OFFSET;

    return true;
}

const ImuData* hal_imu_get_data(void) {
    return &s_data;
}

const ImuRawData* hal_imu_get_raw(void) {
    return &s_raw;
}

float hal_imu_gyro_z_dps(void) {
    return s_data.gyroZ_dps;
}

int16_t hal_imu_gyro_z_dps10(void) {
    // ×10 for 0.1°/s resolution on BLE wire
    float v = s_data.gyroZ_dps * 10.0f;
    if (v >  32767.0f) return 32767;
    if (v < -32768.0f) return -32768;
    return (int16_t)v;
}

void hal_imu_calibrate(uint16_t samples) {
    if (!s_init || samples == 0) return;

    double sum_gx = 0, sum_gy = 0, sum_gz = 0;
    double sum_ax = 0, sum_ay = 0, sum_az = 0;
    uint16_t good = 0;

    for (uint16_t i = 0; i < samples; i++) {
        if (hal_imu_update()) {
            sum_gx += (float)s_raw.gyroX / s_gyro_sensitivity;
            sum_gy += (float)s_raw.gyroY / s_gyro_sensitivity;
            sum_gz += (float)s_raw.gyroZ / s_gyro_sensitivity;
            sum_ax += (float)s_raw.accelX / ACCEL_SENSITIVITY;
            sum_ay += (float)s_raw.accelY / ACCEL_SENSITIVITY;
            sum_az += (float)s_raw.accelZ / ACCEL_SENSITIVITY;
            good++;
        }
        delay(2);   // ~500 Hz sampling during calibration
    }

    if (good > 0) {
        s_gyro_off_x  = (float)(sum_gx / good);
        s_gyro_off_y  = (float)(sum_gy / good);
        s_gyro_off_z  = (float)(sum_gz / good);
        s_accel_off_x = (float)(sum_ax / good);
        s_accel_off_y = (float)(sum_ay / good);
        // Z offset: subtract 1g (assuming sensor is level, Z reads ~1g)
        s_accel_off_z = (float)(sum_az / good) - 1.0f;
        s_calibrated  = true;
    }
}

bool hal_imu_is_init(void) { return s_init; }

bool  hal_imu_is_calibrated(void)     { return s_calibrated; }
float hal_imu_get_gyro_offset_x(void) { return s_gyro_off_x; }
float hal_imu_get_gyro_offset_y(void) { return s_gyro_off_y; }
float hal_imu_get_gyro_offset_z(void) { return s_gyro_off_z; }
