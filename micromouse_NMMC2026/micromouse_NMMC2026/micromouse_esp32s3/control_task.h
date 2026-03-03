// ═══════════════════════════════════════════════════════════════════════════════
//  control_task.h — Real-Time Control Task (Core1, 1kHz)
//
//  Layer: Core Infrastructure
//  Dependencies: config.h, protocol.h, comms_state.h, all HALs
//
//  ★ Iron Rules:
//    - Timing: esp_timer (dispatch_method=ESP_TIMER_TASK) + xTaskNotifyGive()
//      ControlTask blocks on ulTaskNotifyTake() → wakes every 1ms
//    - FORBIDDEN: vTaskNotifyGiveFromISR, portYIELD_FROM_ISR, vTaskDelayUntil
//    - Single writer: only ControlTask modifies motor/PID state
//    - CMD apply: dequeue from g_cmdQ at tick boundary only
//    - SharedSnapshot: write under portMUX after each step
//    - No I2C or slow operations in step1kHz (delegate to SensorTask)
// ═══════════════════════════════════════════════════════════════════════════════
#ifndef CONTROL_TASK_H
#define CONTROL_TASK_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ─────────────────────────────────────────────────────────────────────────────
// Start ControlTask — call from setup() after HAL init + comms_state_init
// Creates esp_timer + FreeRTOS task pinned to Core1.
// ─────────────────────────────────────────────────────────────────────────────
void control_task_start(void);

#ifdef __cplusplus
}
#endif

#endif // CONTROL_TASK_H
