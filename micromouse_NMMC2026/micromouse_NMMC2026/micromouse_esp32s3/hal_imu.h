// ═══════════════════════════════════════════════════════════════════════════════
//  hal_imu.h — IMU HAL (MPU6050 / GY-521)
//
//  Layer: HAL
//  Dependencies: config.h, Wire (I2C Bus 0)
//
//  Design:
//    - Direct register access via Wire (no external library needed)
//    - Provides calibrated gyro (°/s) and raw accel
//    - Calibration: measures offset while stationary
//    - GyroZ is primary sensor for heading control
//    - ★ I2C reads are SLOW (~300µs) — do NOT call in step1kHz()
//      Instead, call hal_imu_update() from SensorTask or decimate in control
// ═══════════════════════════════════════════════════════════════════════════════
#ifndef HAL_IMU_H
#define HAL_IMU_H

#include <stdint.h>
#include <stdbool.h>

// Raw IMU data (integers, before calibration/scaling)
typedef struct {
    int16_t accelX, accelY, accelZ;
    int16_t gyroX, gyroY, gyroZ;
    int16_t temp_raw;
} ImuRawData;

// Calibrated IMU data (floating point, physical units)
typedef struct {
    float gyroX_dps;    // °/s
    float gyroY_dps;
    float gyroZ_dps;    // primary for heading
    float accelX_g;     // g (9.81 m/s²)
    float accelY_g;
    float accelZ_g;
    float temp_C;       // temperature (°C)
} ImuData;

// ── Initialization ──────────────────────────────────────────────────────────
// Starts I2C Bus 0 and configures MPU6050.
// Returns true if WHO_AM_I register reads correctly.
bool hal_imu_init(void);

// ── Update (call at sensor rate, e.g. 500Hz–1kHz from SensorTask) ───────────
// Reads all 14 registers in one I2C burst (~800µs).
// Updates internal calibrated data.
bool hal_imu_update(void);

// ── Get latest calibrated data ──────────────────────────────────────────────
const ImuData* hal_imu_get_data(void);

// ── Get raw data (for debugging / Lab 1) ────────────────────────────────────
const ImuRawData* hal_imu_get_raw(void);

// ── Quick access: calibrated gyro Z (°/s) ───────────────────────────────────
float hal_imu_gyro_z_dps(void);

// ── Quick access: gyro Z × 10 as int16 (for BLE FAST packet) ───────────────
int16_t hal_imu_gyro_z_dps10(void);

// ── Calibration ─────────────────────────────────────────────────────────────
// Robot must be STATIONARY. Samples N readings and computes offset.
// Blocks for ~(samples × 2) ms. Typical: 500 samples = ~1 second.
void hal_imu_calibrate(uint16_t samples);

// ── Get calibration results (for GUI display / export) ──────────────────────
bool  hal_imu_is_calibrated(void);
float hal_imu_get_gyro_offset_x(void);
float hal_imu_get_gyro_offset_y(void);
float hal_imu_get_gyro_offset_z(void);   // most important for micromouse

// ── Status ──────────────────────────────────────────────────────────────────
bool hal_imu_is_init(void);

#endif // HAL_IMU_H
