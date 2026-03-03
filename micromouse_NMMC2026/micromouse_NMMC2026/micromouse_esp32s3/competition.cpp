// ═══════════════════════════════════════════════════════════════════════════════
//  competition.cpp — Standalone Competition Controller
//
//  Uses DIP switches + push button + LED for autonomous operation.
//  Coexists with BLE debug mode (DIP=0 → BLE mode, competition inactive).
//
//  Integration: add to control_task.cpp:
//    #include "competition.h"
//    // In setup:    competition_init();
//    // In tick:     if (competition_is_active()) {
//    //                  if (competition_tick(&vL, &vR, velL, velR, gyro))
//    //                      { s_vCmdL_mV = vL; s_vCmdR_mV = vR; }
//    //              }
// ═══════════════════════════════════════════════════════════════════════════════
#include "competition.h"
#include "lab5_search.h"
#include "lab6_speedrun.h"
#include "hal_ui.h"
#include "hal_buzzer.h"
#include "config.h"
#include <string.h>

// ── State ────────────────────────────────────────────────────────────────────
static CompState  s_state    = COMP_IDLE;
static uint8_t    s_mode     = COMP_MODE_BLE_DEBUG;
static uint8_t    s_speed_lv = 2;  // speed level for speed run

// Button debouncing
static bool       s_btn_prev     = false;
static uint32_t   s_btn_down_ms  = 0;     // how long button is held
static bool       s_btn_handled  = false;  // consumed this press?
#define BTN_DEBOUNCE_MS   50
#define BTN_LONG_PRESS_MS 2000

// Timing
static uint32_t   s_tick_count     = 0;
static uint32_t   s_wait_timer     = 0;
#define COMP_PAUSE_MS    3000   // pause between search done and speed run (full comp)

// LED blink
static uint8_t s_blink_counter = 0;

// Forward declarations for lab interfaces
// (lab5 and lab6 are controlled via their existing handle_test/tick functions)
// We'll create synthetic CommandRequest/Result to drive them

static void start_search(uint8_t mode);
static void start_speedrun(uint8_t level);
static void start_return(void);
static void start_wall_follow(int8_t side);
static void comp_stop_all(void);
static void update_led(void);
static void short_beep(void);
static void long_beep(void);

// ═══════════════════════════════════════════════════════════════════════════════
// Init
// ═══════════════════════════════════════════════════════════════════════════════
void competition_init(void) {
    // Read DIP switches to determine mode
    uint8_t dip = hal_ui_dip_read();      // returns 4-bit value
    s_mode = dip & 0x0F;

    // Determine speed level for speed run modes
    switch (s_mode) {
        case COMP_MODE_SPEED_LV0: s_speed_lv = 0; break;
        case COMP_MODE_SPEED_LV1: s_speed_lv = 1; break;
        case COMP_MODE_SPEED_LV2: s_speed_lv = 2; break;
        case COMP_MODE_SPEED_LV3: s_speed_lv = 3; break;
        case COMP_MODE_FULL_COMP: s_speed_lv = 2; break;  // default for competition
        default: s_speed_lv = 0; break;
    }

    s_state = COMP_IDLE;
    s_tick_count = 0;
}

// ═══════════════════════════════════════════════════════════════════════════════
// Button handling (call from tick at 1kHz)
// ═══════════════════════════════════════════════════════════════════════════════
typedef enum { BTN_NONE, BTN_SHORT, BTN_LONG } BtnEvent;

static BtnEvent poll_button(void) {
    bool pressed = hal_ui_btn_start();  // true = pressed

    if (pressed && !s_btn_prev) {
        // Rising edge — button just pressed
        s_btn_down_ms = 0;
        s_btn_handled = false;
    }

    if (pressed) {
        s_btn_down_ms++;
        if (s_btn_down_ms >= BTN_LONG_PRESS_MS && !s_btn_handled) {
            s_btn_handled = true;
            s_btn_prev = pressed;
            return BTN_LONG;
        }
    } else if (s_btn_prev && !pressed) {
        // Falling edge — button released
        s_btn_prev = false;
        if (!s_btn_handled && s_btn_down_ms >= BTN_DEBOUNCE_MS) {
            return BTN_SHORT;
        }
        return BTN_NONE;
    }

    s_btn_prev = pressed;
    return BTN_NONE;
}

// ═══════════════════════════════════════════════════════════════════════════════
// Tick (called at 1kHz from control_task)
// ═══════════════════════════════════════════════════════════════════════════════
bool competition_tick(int16_t* vCmdL_mV, int16_t* vCmdR_mV,
                      int16_t velL_mmps, int16_t velR_mmps,
                      float gyroZ_dps) {

    if (s_mode == COMP_MODE_BLE_DEBUG) return false;

    s_tick_count++;
    BtnEvent btn = poll_button();

    // Long press always resets
    if (btn == BTN_LONG) {
        comp_stop_all();
        s_state = COMP_IDLE;
        long_beep();
        *vCmdL_mV = 0; *vCmdR_mV = 0;
        update_led();
        return true;
    }

    switch (s_state) {

    // ── IDLE: waiting for button ─────────────────────────────────────────
    case COMP_IDLE:
        *vCmdL_mV = 0; *vCmdR_mV = 0;
        if (btn == BTN_SHORT) {
            s_state = COMP_ARMED;
            short_beep();
        }
        break;

    // ── ARMED: ready to go, press button to start ────────────────────────
    case COMP_ARMED:
        *vCmdL_mV = 0; *vCmdR_mV = 0;
        if (btn == BTN_SHORT) {
            short_beep();
            // Start the appropriate action
            switch (s_mode) {
                case COMP_MODE_SEARCH_SG:
                    start_search(0);
                    s_state = COMP_SEARCHING;
                    break;
                case COMP_MODE_SEARCH_CONT:
                    start_search(1);
                    s_state = COMP_SEARCHING;
                    break;
                case COMP_MODE_SPEED_LV0:
                case COMP_MODE_SPEED_LV1:
                case COMP_MODE_SPEED_LV2:
                case COMP_MODE_SPEED_LV3:
                    start_speedrun(s_speed_lv);
                    s_state = COMP_SPEEDRUN;
                    break;
                case COMP_MODE_FULL_COMP:
                    start_search(1);  // continuous mode for competition
                    s_state = COMP_SEARCHING;
                    break;
                case COMP_MODE_WALL_FOLLOW_L:
                    start_wall_follow(-1);
                    s_state = COMP_SEARCHING;  // reuse state for display
                    break;
                case COMP_MODE_WALL_FOLLOW_R:
                    start_wall_follow(+1);
                    s_state = COMP_SEARCHING;
                    break;
                default:
                    s_state = COMP_ERROR;
                    break;
            }
        }
        break;

    // ── SEARCHING: lab5 running ──────────────────────────────────────────
    case COMP_SEARCHING: {
        bool running = lab5_tick(vCmdL_mV, vCmdR_mV,
                                  velL_mmps, velR_mmps, gyroZ_dps);
        if (!running) {
            SearchState ss = lab5_get_state();
            if (ss == SEARCH_DONE) {
                // Search complete — what's next?
                if (s_mode == COMP_MODE_FULL_COMP) {
                    // Full competition: wait, then speed run
                    s_state = COMP_WAIT_SPEEDRUN;
                    s_wait_timer = 0;
                    short_beep();
                } else {
                    s_state = COMP_DONE;
                    long_beep();
                }
            } else if (ss == SEARCH_GOAL_REACHED) {
                // Goal found — auto-return in full competition mode
                if (s_mode == COMP_MODE_FULL_COMP ||
                    s_mode == COMP_MODE_SEARCH_SG ||
                    s_mode == COMP_MODE_SEARCH_CONT) {
                    // Lab5 auto-handles return when s_continuous=true
                    // Keep running
                    s_state = COMP_RETURNING;
                }
            } else if (ss == SEARCH_ERROR) {
                s_state = COMP_ERROR;
            } else {
                // IDLE from step or wall follow stop
                s_state = COMP_DONE;
            }
        }
        break;
    }

    // ── RETURNING: lab5 returning to start ───────────────────────────────
    case COMP_RETURNING: {
        bool running = lab5_tick(vCmdL_mV, vCmdR_mV,
                                  velL_mmps, velR_mmps, gyroZ_dps);
        if (!running) {
            SearchState ss = lab5_get_state();
            if (ss == SEARCH_DONE) {
                if (s_mode == COMP_MODE_FULL_COMP) {
                    s_state = COMP_WAIT_SPEEDRUN;
                    s_wait_timer = 0;
                    short_beep();
                } else {
                    s_state = COMP_DONE;
                    long_beep();
                }
            } else if (ss == SEARCH_ERROR) {
                s_state = COMP_ERROR;
            }
        }
        break;
    }

    // ── WAIT_SPEEDRUN: pause before speed run (full comp) ────────────────
    case COMP_WAIT_SPEEDRUN:
        *vCmdL_mV = 0; *vCmdR_mV = 0;
        s_wait_timer++;
        // Auto-start after pause, OR button press starts immediately
        if (s_wait_timer >= COMP_PAUSE_MS || btn == BTN_SHORT) {
            start_speedrun(s_speed_lv);
            s_state = COMP_SPEEDRUN;
            short_beep();
        }
        break;

    // ── SPEEDRUN: lab6 running ───────────────────────────────────────────
    case COMP_SPEEDRUN: {
        bool running = lab6_tick(vCmdL_mV, vCmdR_mV,
                                  velL_mmps, velR_mmps, gyroZ_dps);
        if (!running) {
            s_state = COMP_DONE;
            long_beep();
        }
        break;
    }

    // ── DONE: all finished ───────────────────────────────────────────────
    case COMP_DONE:
        *vCmdL_mV = 0; *vCmdR_mV = 0;
        if (btn == BTN_SHORT) {
            // Allow restart
            s_state = COMP_ARMED;
            short_beep();
        }
        break;

    // ── ERROR ────────────────────────────────────────────────────────────
    case COMP_ERROR:
        *vCmdL_mV = 0; *vCmdR_mV = 0;
        if (btn == BTN_SHORT) {
            s_state = COMP_IDLE;
        }
        break;
    }

    update_led();
    return true;
}

// ═══════════════════════════════════════════════════════════════════════════════
// Helper: start search via lab5
// ═══════════════════════════════════════════════════════════════════════════════
static void start_search(uint8_t mode) {
    // Build synthetic command for lab5
    CommandRequest cmd;
    CommandResult  res;
    int16_t dummy_l = 0, dummy_r = 0;
    memset(&cmd, 0, sizeof(cmd));
    cmd.opcode = 0x30;  // CMD_RUN_TEST
    cmd.req_id = 0;
    cmd.payload[0] = 0x50;  // TEST_SEARCH_RUN
    cmd.payload[1] = mode;
    lab5_handle_test(&cmd, &res, &dummy_l, &dummy_r);
}

static void start_return(void) {
    CommandRequest cmd;
    CommandResult  res;
    int16_t dummy_l = 0, dummy_r = 0;
    memset(&cmd, 0, sizeof(cmd));
    cmd.opcode = 0x30;
    cmd.req_id = 0;
    cmd.payload[0] = 0x52;  // TEST_SEARCH_RETURN
    lab5_handle_test(&cmd, &res, &dummy_l, &dummy_r);
}

static void start_speedrun(uint8_t level) {
    CommandRequest cmd;
    CommandResult  res;
    int16_t dummy_l = 0, dummy_r = 0;
    memset(&cmd, 0, sizeof(cmd));
    cmd.opcode = 0x30;
    cmd.req_id = 0;
    cmd.payload[0] = 0x60;  // TEST_SPEEDRUN_START
    cmd.payload[1] = level;
    lab6_handle_test(&cmd, &res, &dummy_l, &dummy_r);
}

static void start_wall_follow(int8_t side) {
    CommandRequest cmd;
    CommandResult  res;
    int16_t dummy_l = 0, dummy_r = 0;
    memset(&cmd, 0, sizeof(cmd));
    cmd.opcode = 0x30;
    cmd.req_id = 0;
    cmd.payload[0] = (side < 0) ? 0x57 : 0x58;  // TEST_WALL_FOLLOW_L/R
    lab5_handle_test(&cmd, &res, &dummy_l, &dummy_r);
}

static void comp_stop_all(void) {
    lab5_stop();
    lab6_stop();
}

// ═══════════════════════════════════════════════════════════════════════════════
// LED feedback
// ═══════════════════════════════════════════════════════════════════════════════
static void update_led(void) {
    s_blink_counter++;
    bool blink = (s_blink_counter & 0x80) != 0;  // ~4Hz blink at 1kHz/256

    switch (s_state) {
        case COMP_IDLE:
            hal_ui_led_set(0, 0, 0);                       // off
            break;
        case COMP_ARMED:
            hal_ui_led_set(0, 0, blink ? 40 : 10);         // blue pulse
            break;
        case COMP_SEARCHING:
            hal_ui_led_set(40, 30, 0);                      // yellow
            break;
        case COMP_SEARCH_DONE:
            hal_ui_led_set(0, blink ? 40 : 10, 0);         // green blink
            break;
        case COMP_RETURNING:
            hal_ui_led_set(0, 30, 40);                      // cyan
            break;
        case COMP_WAIT_SPEEDRUN:
            hal_ui_led_set(blink ? 30 : 5, blink ? 30 : 5, blink ? 30 : 5);  // white blink
            break;
        case COMP_SPEEDRUN:
            hal_ui_led_set(40, 0, 40);                      // magenta
            break;
        case COMP_DONE:
            hal_ui_led_set(0, 40, 0);                       // green solid
            break;
        case COMP_ERROR:
            hal_ui_led_set(blink ? 40 : 0, 0, 0);          // red blink
            break;
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
// Sound feedback
// ═══════════════════════════════════════════════════════════════════════════════
static void short_beep(void) {
    hal_buzzer_tone(2000, 100);  // 2kHz, 100ms
}

static void long_beep(void) {
    hal_buzzer_tone(1500, 500);  // 1.5kHz, 500ms
}

// ═══════════════════════════════════════════════════════════════════════════════
// Query
// ═══════════════════════════════════════════════════════════════════════════════
bool competition_is_active(void) {
    return s_mode != COMP_MODE_BLE_DEBUG;
}

CompState competition_get_state(void) {
    return s_state;
}

uint8_t competition_get_mode(void) {
    return s_mode;
}
