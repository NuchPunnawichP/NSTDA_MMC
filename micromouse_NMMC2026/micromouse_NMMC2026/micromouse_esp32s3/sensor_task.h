// ═══════════════════════════════════════════════════════════════════════════════
//  sensor_task.h — Sensor Reading Task (Core1)
//
//  Layer: Core Infrastructure
//  Dependencies: hal_imu.h, hal_wallsensor.h
//
//  Design:
//    - Runs on Core1 at medium priority (below ControlTask)
//    - Reads IMU at ~500Hz (every 2ms) — fast enough for gyro integration
//    - Reads wall sensors at ~30Hz (every ~33ms) — I2C/ADC limited
//    - Stores results accessible to ControlTask (same core, no lock needed)
//    - ★ I2C reads are slow (~300-800µs) — MUST NOT be in step1kHz()
// ═══════════════════════════════════════════════════════════════════════════════
#ifndef SENSOR_TASK_H
#define SENSOR_TASK_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Start SensorTask — call from setup() after HAL init
void sensor_task_start(void);

// Latest wall sensor distances (mm) — updated by SensorTask
// Safe to read from ControlTask (same core)
uint16_t sensor_get_wall_mm(uint8_t position);
void     sensor_get_wall_all(uint16_t out[4]);
uint8_t  sensor_get_wall_bitmap(void);

// ── Async Gyro Calibration (non-blocking from ControlTask) ──────────────────
// Request: sets flag, SensorTask will run calibration in background
// Status:  0=idle, 1=in_progress, 2=done
void    sensor_request_gyro_cal(uint16_t samples);
uint8_t sensor_gyro_cal_status(void);  // 0=idle, 1=busy, 2=done

// ── Async Wall Calibration (non-blocking) ───────────────────────────────────
void    sensor_request_wall_cal(uint16_t samples);
uint8_t sensor_wall_cal_status(void);  // 0=idle, 1=busy, 2=done

#ifdef __cplusplus
}
#endif

#endif // SENSOR_TASK_H
