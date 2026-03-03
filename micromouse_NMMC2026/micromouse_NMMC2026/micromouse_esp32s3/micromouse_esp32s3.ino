// ═══════════════════════════════════════════════════════════════════════════════
//  micromouse_esp32s3.ino — Main Orchestrator
//
//  Responsibilities:
//    setup(): init HAL → init comms → init BLE → start tasks → done
//    loop():  heartbeat LED + button poll only (LOW priority, no control)
//
//  ★ Iron Rules enforced here:
//    - loop() does NOT touch timing, motor state, or BLE notify
//    - All control is in ControlTask (Core1)
//    - All comms is in CommsTask (Core0)
//    - This file does NOT #include BLEDevice.h (it's in ble_service.cpp only)
// ═══════════════════════════════════════════════════════════════════════════════

#include "config.h"
#include "protocol.h"
#include <esp_mac.h>   // for esp_efuse_mac_get_default()

// HAL
#include "hal_motor.h"
#include "hal_encoder.h"
#include "hal_battery.h"
#include "hal_imu.h"
#include "hal_wallsensor.h"
#include "hal_ui.h"
#include "hal_buzzer.h"

// Core Infrastructure
#include "comms_state.h"
#include "ble_service.h"
#include "comms_task.h"
#include "control_task.h"
#include "sensor_task.h"

// ── BLE device name (prefix + last 4 hex of MAC) ───────────────────────────
static char s_ble_name[16];

static void build_ble_name(void) {
    // Use base MAC (WiFi STA) — BT MAC is base+2, but we just need unique suffix
    uint8_t mac[6];
    esp_efuse_mac_get_default(mac);
    snprintf(s_ble_name, sizeof(s_ble_name), "%s_%02X%02X",
             BLE_DEVICE_NAME_PREFIX, mac[4], mac[5]);
}

// ═══════════════════════════════════════════════════════════════════════════════
// setup() — runs once on Core0 (Arduino default)
// ═══════════════════════════════════════════════════════════════════════════════
void setup() {
    // ── Serial for emergency debug only (not primary debug channel) ──
    Serial.begin(115200);
    delay(100);
    Serial.println("\n[MM] Micromouse ESP32-S3 starting...");

    // ════════════════════════════════════════════════════════════════════
    // Phase 1: HAL Initialization (hardware layer)
    // ════════════════════════════════════════════════════════════════════

    Serial.print("[MM] UI init... ");
    hal_ui_init();
    hal_ui_led_set(0, 0, 50);  // dim blue = booting
    Serial.println("OK");

    Serial.print("[MM] Buzzer init... ");
    hal_buzzer_init();
    Serial.println("OK");

    Serial.print("[MM] Battery init... ");
    hal_battery_init();
    uint16_t vbat = hal_battery_read_mV();
    Serial.printf("OK (%d mV)\n", vbat);

    // Check battery before continuing
    if (vbat < BATT_CRITICAL_MV && vbat > 1000) {
        // Battery present but critically low
        Serial.println("[MM] WARNING: Battery critically low!");
        hal_ui_led_set(255, 0, 0);  // red
        hal_buzzer_beep_error();
        delay(3000);
    }

    Serial.print("[MM] Motor init... ");
    hal_motor_init();
    Serial.println("OK");

    Serial.print("[MM] Encoder init... ");
    bool enc_ok = hal_encoder_init();
    Serial.println(enc_ok ? "OK" : "FAIL");

    Serial.print("[MM] IMU init... ");
    bool imu_ok = hal_imu_init();
    Serial.println(imu_ok ? "OK" : "FAIL");
    // Gyro auto-calibrate moved to sensor_task (non-blocking)

    Serial.print("[MM] Wall sensors init... ");
    bool wall_ok = hal_wall_init();
    Serial.printf("%s (%d active)\n",
                  wall_ok ? "OK" : "FAIL",
                  hal_wall_active_count());

    // ════════════════════════════════════════════════════════════════════
    // Phase 2: Communication Infrastructure
    // ════════════════════════════════════════════════════════════════════

    Serial.print("[MM] Comms state init... ");
    comms_state_init();
    Serial.println("OK");

    Serial.print("[MM] BLE init... ");
    build_ble_name();
    bool ble_ok = ble_service_init(s_ble_name);
    Serial.printf("%s (name: %s)\n", ble_ok ? "OK" : "FAIL", s_ble_name);

    // Set CAPS packet with actual hardware info
    CapsPacket caps;
    proto_build_caps(&caps,
                     hal_wall_active_count(),
                     WALL_SENSOR_TYPE,
                     7);  // 7 labs supported
    // Set feature bits based on what initialized
    uint16_t features = 0;
    if (hal_motor_is_init())   features |= (1 << 0);
    if (enc_ok)                features |= (1 << 1);
    if (imu_ok)                features |= (1 << 2);
    if (wall_ok)               features |= (1 << 3);
    if (ble_ok)                features |= (1 << 4);
    caps.features = features;
    proto_stamp_crc(&caps);  // re-stamp after modifying features
    ble_set_caps(&caps);

    // ════════════════════════════════════════════════════════════════════
    // Phase 3: Start Tasks
    // ════════════════════════════════════════════════════════════════════

    Serial.print("[MM] Starting ControlTask (Core1, 1kHz)... ");
    control_task_start();
    Serial.println("OK");

    Serial.print("[MM] Starting SensorTask (Core1, 500Hz)... ");
    sensor_task_start();
    Serial.println("OK");

    Serial.print("[MM] Starting CommsTask (Core0)... ");
    comms_task_start();
    Serial.println("OK");

    // ════════════════════════════════════════════════════════════════════
    // Phase 4: Ready
    // ════════════════════════════════════════════════════════════════════

    hal_ui_led_battery(hal_battery_get_level());
    hal_buzzer_beep();

    Serial.println("[MM] ═══════════════════════════════════════");
    Serial.println("[MM] System ready. Waiting for BLE connection.");
    Serial.printf("[MM] DIP switch: %d\n", hal_ui_dip_read());
    Serial.println("[MM] ═══════════════════════════════════════");
}

// ═══════════════════════════════════════════════════════════════════════════════
// loop() — LOW priority heartbeat / UI only
//   ★ NO control logic. NO BLE notify. NO motor commands.
// ═══════════════════════════════════════════════════════════════════════════════
void loop() {
    // ── UI: update buttons (debounce) ──
    hal_ui_update();
    hal_buzzer_update();

    // ── Heartbeat LED ──
    // Connected: steady color based on battery
    // Disconnected: slow blink blue
    static uint32_t last_led_ms = 0;
    static bool led_on = true;
    uint32_t now = millis();

    if (comms_get_ble_connected()) {
        // Connected — show battery status color, update every 2s
        if ((now - last_led_ms) > 2000) {
            last_led_ms = now;
            hal_ui_led_battery(hal_battery_get_level());
        }
    } else {
        // Disconnected — blink blue every 1s
        if ((now - last_led_ms) > 1000) {
            last_led_ms = now;
            led_on = !led_on;
            if (led_on) {
                hal_ui_led_set(0, 0, 80);   // dim blue
            } else {
                hal_ui_led_off();
            }
        }
    }

    // ── Button handling (placeholder for DIP mode switching) ──
    if (hal_ui_btn_start_pressed()) {
        // Could trigger mode start based on DIP switch
        hal_buzzer_beep();
    }
    if (hal_ui_btn_mode_pressed()) {
        // Could cycle through modes
        hal_buzzer_beep();
    }

    // ── loop runs at ~20 Hz (50ms delay) — no need to be faster ──
    delay(50);
}
