// ═══════════════════════════════════════════════════════════════════════════════
//  hal_buzzer.cpp — Buzzer HAL Implementation (LEDC tone)
// ═══════════════════════════════════════════════════════════════════════════════
#include "hal_buzzer.h"
#include <Arduino.h>

static bool     s_available = false;
static bool     s_playing   = false;
static uint32_t s_stop_at   = 0;       // millis when tone should stop

void hal_buzzer_init(void) {
#if PIN_BUZZER >= 0
    s_available = true;
    // Don't attach LEDC yet — only when playing
    pinMode(PIN_BUZZER, OUTPUT);
    digitalWrite(PIN_BUZZER, LOW);
#else
    s_available = false;
#endif
}

void hal_buzzer_tone(uint16_t freq_hz, uint16_t duration_ms) {
    if (!s_available || freq_hz == 0) return;
#if PIN_BUZZER >= 0
    // Use Arduino tone() which uses LEDC internally on ESP32
    tone(PIN_BUZZER, freq_hz, duration_ms);
    s_playing = true;
    s_stop_at = millis() + duration_ms;
#endif
}

void hal_buzzer_update(void) {
    if (!s_playing) return;
    if (millis() >= s_stop_at) {
        hal_buzzer_stop();
    }
}

void hal_buzzer_stop(void) {
#if PIN_BUZZER >= 0
    if (s_available) {
        noTone(PIN_BUZZER);
        digitalWrite(PIN_BUZZER, LOW);
    }
#endif
    s_playing = false;
}

void hal_buzzer_beep(void) {
    hal_buzzer_tone(2000, 100);
}

void hal_buzzer_beep_ok(void) {
    // Double beep: two short high tones
    hal_buzzer_tone(2500, 80);
    // Note: second beep handled by caller or queue — keep simple for now
}

void hal_buzzer_beep_error(void) {
    hal_buzzer_tone(400, 500);
}

void hal_buzzer_beep_arm(void) {
    hal_buzzer_tone(1500, 150);
}

void hal_buzzer_beep_disarm(void) {
    hal_buzzer_tone(800, 200);
}
