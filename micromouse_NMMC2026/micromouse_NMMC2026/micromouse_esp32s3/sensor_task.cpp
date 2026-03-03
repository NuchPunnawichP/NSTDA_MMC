// ═══════════════════════════════════════════════════════════════════════════════
//  sensor_task.cpp — Sensor Reading Task Implementation
// ═══════════════════════════════════════════════════════════════════════════════
#include "sensor_task.h"
#include "config.h"
#include "hal_imu.h"
#include "hal_wallsensor.h"
#include "hal_ui.h"
#include "hal_buzzer.h"

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// ── Cached wall data (same-core access from ControlTask, no lock needed) ────
static uint16_t s_wall_mm[4] = {0};
static uint8_t  s_wall_bm    = 0;

// ── Async calibration state ─────────────────────────────────────────────────
static volatile uint8_t  s_gyro_cal_status  = 0;  // 0=idle, 1=busy, 2=done
static volatile uint16_t s_gyro_cal_samples = 0;
static volatile uint8_t  s_wall_cal_status  = 0;
static volatile uint16_t s_wall_cal_samples = 0;

// ── Task function ───────────────────────────────────────────────────────────
static void sensorTaskFunc(void* param) {
    (void)param;

    // ── Auto-calibrate gyro at startup (runs in this task, won't block control) ──
    if (hal_imu_is_init()) {
        hal_ui_led_set(0, 50, 50);  // cyan = calibrating
        hal_imu_calibrate(500);     // ~1s, blocks only this task
        hal_buzzer_beep_ok();       // double beep = done
        hal_ui_led_set(0, 80, 0);  // green flash
        vTaskDelay(pdMS_TO_TICKS(300));
        hal_ui_led_off();
    }

    uint32_t imu_count  = 0;
    uint32_t wall_count = 0;

    for (;;) {
        // ── Check for gyro calibration request ──
        if (s_gyro_cal_status == 1) {
            // Run calibration (blocks sensor task only, NOT control task)
            hal_imu_calibrate(s_gyro_cal_samples);
            s_gyro_cal_status = 2;  // done
            // Skip this tick's normal reads
            vTaskDelay(pdMS_TO_TICKS(2));
            continue;
        }

        // ── Check for wall calibration request ──
        if (s_wall_cal_status == 1) {
            hal_wall_calibrate(s_wall_cal_samples);
            s_wall_cal_status = 2;  // done
            vTaskDelay(pdMS_TO_TICKS(2));
            continue;
        }

        // ── IMU: read every tick (~2ms → 500Hz) ──
        if (hal_imu_is_init()) {
            hal_imu_update();
        }
        imu_count++;

        // ── Wall sensors: read every ~16 ticks (~32ms → ~30Hz) ──
        if (++wall_count >= 16) {
            wall_count = 0;
            if (hal_wall_is_init()) {
                hal_wall_update();
                hal_wall_get_all(s_wall_mm);
                s_wall_bm = hal_wall_bitmap();
            }
        }

        // Sleep 2ms base rate
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

// ── Public API ──────────────────────────────────────────────────────────────

void sensor_task_start(void) {
    xTaskCreatePinnedToCore(
        sensorTaskFunc,
        "SensorTask",
        STACK_SENSOR,
        NULL,
        PRIO_SENSOR,
        NULL,
        CORE_SENSOR     // Core1, same as ControlTask
    );
}

uint16_t sensor_get_wall_mm(uint8_t pos) {
    if (pos >= 4) return 0;
    return s_wall_mm[pos];
}

void sensor_get_wall_all(uint16_t out[4]) {
    for (int i = 0; i < 4; i++) out[i] = s_wall_mm[i];
}

uint8_t sensor_get_wall_bitmap(void) {
    return s_wall_bm;
}

void sensor_request_gyro_cal(uint16_t samples) {
    s_gyro_cal_samples = samples;
    s_gyro_cal_status  = 1;  // request
}

uint8_t sensor_gyro_cal_status(void) {
    return s_gyro_cal_status;
}

void sensor_request_wall_cal(uint16_t samples) {
    s_wall_cal_samples = samples;
    s_wall_cal_status  = 1;  // request
}

uint8_t sensor_wall_cal_status(void) {
    return s_wall_cal_status;
}
