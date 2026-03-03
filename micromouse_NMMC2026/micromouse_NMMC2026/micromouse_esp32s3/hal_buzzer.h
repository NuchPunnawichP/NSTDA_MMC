// ═══════════════════════════════════════════════════════════════════════════════
//  hal_buzzer.h — Buzzer HAL (optional, for audio feedback)
//
//  Layer: HAL
//  Uses LEDC for tone generation. Non-blocking.
//  ★ If no buzzer hardware, all calls are safe no-ops.
// ═══════════════════════════════════════════════════════════════════════════════
#ifndef HAL_BUZZER_H
#define HAL_BUZZER_H

#include <stdint.h>
#include <stdbool.h>

// Set buzzer pin in config.h. Define -1 if no buzzer.
#ifndef PIN_BUZZER
#define PIN_BUZZER              -1      // -1 = no buzzer installed
#endif

void hal_buzzer_init(void);

// Play tone at frequency (Hz) for duration_ms. Non-blocking: tone starts,
// call hal_buzzer_update() periodically to auto-stop after duration.
void hal_buzzer_tone(uint16_t freq_hz, uint16_t duration_ms);

// Predefined feedback sounds
void hal_buzzer_beep(void);             // short 100ms beep
void hal_buzzer_beep_ok(void);          // double beep (success)
void hal_buzzer_beep_error(void);       // long low beep (error)
void hal_buzzer_beep_arm(void);         // ascending tone (armed)
void hal_buzzer_beep_disarm(void);      // descending tone (disarmed)

// Call periodically (e.g. from UI task) to handle auto-stop
void hal_buzzer_update(void);

// Stop immediately
void hal_buzzer_stop(void);

#endif // HAL_BUZZER_H
