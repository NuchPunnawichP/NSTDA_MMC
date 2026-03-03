// ═══════════════════════════════════════════════════════════════════════════════
//  lab2b_motor_char.cpp — Lab 2b: Motor Characterization Implementation
//
//  All timed tests managed by lab2b_tick() called from step1kHz().
//  FAST streaming captures data — GUI records + analyzes.
// ═══════════════════════════════════════════════════════════════════════════════
#include "lab2b_motor_char.h"
#include "config.h"
#include "hal_motor.h"
#include "hal_encoder.h"
#include <string.h>

// ── Helpers ─────────────────────────────────────────────────────────────────
static inline int16_t rd_i16(const uint8_t* p) {
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}
static inline uint16_t rd_u16(const uint8_t* p) {
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}
static inline void wr_i16(uint8_t* p, int16_t v) {
    p[0] = (uint8_t)(v); p[1] = (uint8_t)(v >> 8);
}

// ═══════════════════════════════════════════════════════════════════════════════
// Test State Machine
// ═══════════════════════════════════════════════════════════════════════════════
enum Lab2bMode {
    L2B_IDLE = 0,
    L2B_STEP_RESPONSE,
    L2B_RAMP,
    L2B_FIND_BIAS,
};

static Lab2bMode s_mode       = L2B_IDLE;
static uint8_t   s_select     = 0;         // 0=both, 1=L, 2=R

// ── Step Response state ──
static int16_t   s_step_mV    = 0;
static uint32_t  s_step_remain = 0;        // ticks remaining

// ── Ramp state ──
static int16_t   s_ramp_current_mV = 0;
static int16_t   s_ramp_end_mV   = 0;
static int16_t   s_ramp_step_mV  = 0;
static uint32_t  s_ramp_step_ticks = 0;    // ticks per step
static uint32_t  s_ramp_tick_count = 0;    // ticks at current step

// ── Find Bias state ──
static int16_t   s_bias_mV      = 0;       // current probe voltage
static uint32_t  s_bias_settle  = 0;       // ticks to settle
static int32_t   s_bias_enc_start = 0;     // encoder start for detection
static uint8_t   s_bias_phase   = 0;       // 0=left, 1=right, 2=done
static int16_t   s_bias_result_L = 0;
static int16_t   s_bias_result_R = 0;

#define BIAS_START_MV       2000    // start probing from this voltage
#define BIAS_STEP_MV        100     // increment per probe
#define BIAS_MAX_MV         7000    // give up above this
#define BIAS_SETTLE_MS      300     // wait time at each voltage
#define BIAS_THRESHOLD      40       // encoder counts to confirm movement

// ── Apply voltage to selected motor(s) ──
static void apply_voltage(int16_t mV, int16_t* vL, int16_t* vR) {
    switch (s_select) {
        case 0: *vL = mV; *vR = mV; break;
        case 1: *vL = mV; *vR = 0;  break;
        case 2: *vL = 0;  *vR = mV; break;
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
// Test Handlers
// ═══════════════════════════════════════════════════════════════════════════════

// ── 0x21: Step Response ─────────────────────────────────────────────────────
//  payload: [0]=test_id, [1..2]=mV(i16), [3..4]=duration_ms(u16), [5]=select
static void handle_step_response(const uint8_t* payload, CommandResult* res,
                                 int16_t* vL, int16_t* vR) {
    int16_t  mV      = rd_i16(&payload[1]);
    uint16_t dur_ms  = rd_u16(&payload[3]);
    uint8_t  sel     = payload[5];

    if (dur_ms == 0 || dur_ms > 10000) { res->status = RSP_ERR_BAD_ARG; return; }
    if (mV < -VCMD_MAX_MV || mV > VCMD_MAX_MV) { res->status = RSP_ERR_BAD_ARG; return; }
    if (sel > 2) sel = 0;

    s_mode       = L2B_STEP_RESPONSE;
    s_step_mV    = mV;
    s_step_remain= dur_ms;
    s_select     = sel;

    apply_voltage(mV, vL, vR);
    res->status = RSP_OK;
}


// ── 0x20: Ramp Up ──────────────────────────────────────────────────────────
//  payload: [0]=test_id, [1..2]=start_mV(i16), [3..4]=end_mV(i16),
//           [5..6]=step_mV(i16), [7..8]=step_ms(u16), [9]=select
static void handle_ramp_up(const uint8_t* payload, CommandResult* res,
                            int16_t* vL, int16_t* vR) {
    int16_t  start  = rd_i16(&payload[1]);
    int16_t  end    = rd_i16(&payload[3]);
    int16_t  step   = rd_i16(&payload[5]);
    uint16_t step_ms = rd_u16(&payload[7]);
    uint8_t  sel    = payload[9];

    // Validate
    if (step == 0 || step_ms < 50 || step_ms > 5000) {
        res->status = RSP_ERR_BAD_ARG; return;
    }
    if (start < -VCMD_MAX_MV || end < -VCMD_MAX_MV ||
        start > VCMD_MAX_MV  || end > VCMD_MAX_MV) {
        res->status = RSP_ERR_BAD_ARG; return;
    }
    if (sel > 2) sel = 0;

    // Ensure step direction matches start→end
    if (start < end && step < 0) step = -step;
    if (start > end && step > 0) step = -step;

    s_mode = L2B_RAMP;
    s_ramp_current_mV = start;
    s_ramp_end_mV = end;
    s_ramp_step_mV = step;
    s_ramp_step_ticks = step_ms;  // 1 tick = 1ms
    s_ramp_tick_count = 0;
    s_select = sel;

    apply_voltage(start, vL, vR);
    res->status = RSP_OK;
}

// ── 0x22: Find Bias ────────────────────────────────────────────────────────
//  No params needed. Auto-sweep both motors sequentially.
//  RSP (when complete): biasL_mV(i16), biasR_mV(i16) = 4 bytes
// static void handle_find_bias(CommandResult* res, int16_t* vL, int16_t* vR) {
//     s_mode = L2B_FIND_BIAS;
//     s_bias_phase = 0;           // start with left motor
//     s_bias_mV = BIAS_START_MV;
//     s_bias_settle = BIAS_SETTLE_MS;
//     s_bias_result_L = 0;
//     s_bias_result_R = 0;
//     s_select = 1;               // left first

//     // Reset encoder for clean detection
//     hal_encoder_reset();
//     s_bias_enc_start = 0;

//     apply_voltage(s_bias_mV, vL, vR);
//     res->status = RSP_OK;       // ACK start; final result comes as separate RSP
// }
static void handle_find_bias(CommandResult* res, int16_t* vL, int16_t* vR) {
    s_mode = L2B_FIND_BIAS;

    // Start with LEFT motor
    s_bias_phase   = 0;
    s_select       = 1;

    // Reset results
    s_bias_result_L = 0;
    s_bias_result_R = 0;

    // Start scanning from this voltage
    s_bias_mV     = BIAS_START_MV;
    s_bias_settle = BIAS_SETTLE_MS;

    // Reset encoder for clean detection
    hal_encoder_reset();

    // Make sure outputs start from OFF
    *vL = 0;
    *vR = 0;

    // Capture start count (after reset should be 0, but this is the correct pattern)
    s_bias_enc_start = hal_encoder_count_L();

    // Apply first candidate voltage
    apply_voltage(s_bias_mV, vL, vR);

    res->status = RSP_OK;  // ACK start
}

// ═══════════════════════════════════════════════════════════════════════════════
// Public API
// ═══════════════════════════════════════════════════════════════════════════════

bool lab2b_handle_test(const CommandRequest* cmd, CommandResult* result,
                       int16_t* vCmdL_mV, int16_t* vCmdR_mV) {
    uint8_t test_id = cmd->payload[0];

    memset(result, 0, sizeof(*result));
    result->opcode = cmd->opcode;
    result->req_id = cmd->req_id;
    result->status = RSP_OK;

    switch (test_id) {
    case TEST_STEP_RESPONSE:
        handle_step_response(cmd->payload, result, vCmdL_mV, vCmdR_mV);
        return true;
    case TEST_RAMP_UP:
        handle_ramp_up(cmd->payload, result, vCmdL_mV, vCmdR_mV);
        return true;
    case TEST_FIND_BIAS:
        handle_find_bias(result, vCmdL_mV, vCmdR_mV);
        return true;
    default:
        return false;
    }
}

bool lab2b_tick(int16_t* vCmdL_mV, int16_t* vCmdR_mV) {
    switch (s_mode) {

    case L2B_STEP_RESPONSE:
        if (s_step_remain > 0) {
            s_step_remain--;
            apply_voltage(s_step_mV, vCmdL_mV, vCmdR_mV);
            return true;
        }
        // Done
        s_mode = L2B_IDLE;
        *vCmdL_mV = 0; *vCmdR_mV = 0;
        return false;

    case L2B_RAMP:
        s_ramp_tick_count++;
        if (s_ramp_tick_count >= s_ramp_step_ticks) {
            s_ramp_tick_count = 0;
            s_ramp_current_mV += s_ramp_step_mV;

            // Check if ramp complete
            if ((s_ramp_step_mV > 0 && s_ramp_current_mV > s_ramp_end_mV) ||
                (s_ramp_step_mV < 0 && s_ramp_current_mV < s_ramp_end_mV)) {
                s_mode = L2B_IDLE;
                *vCmdL_mV = 0; *vCmdR_mV = 0;
                return false;
            }
        }
        apply_voltage(s_ramp_current_mV, vCmdL_mV, vCmdR_mV);
        return true;

    case L2B_FIND_BIAS: {
        if (s_bias_settle > 0) {
            s_bias_settle--;
            apply_voltage(s_bias_mV, vCmdL_mV, vCmdR_mV);
            return true;
        }

        // Check if wheel moved
        int32_t enc_now;
        if (s_bias_phase == 0) {
            enc_now = hal_encoder_count_L();
        } else {
            enc_now = hal_encoder_count_R();
        }

        int32_t moved = enc_now - s_bias_enc_start;
        if (moved < 0) moved = -moved;

        if (moved >= BIAS_THRESHOLD) {
            // Found bias for current phase
            if (s_bias_phase == 0) {
                s_bias_result_L = s_bias_mV;

                // Switch to right motor
                s_bias_phase = 1;
                s_select = 2;
                s_bias_mV = BIAS_START_MV;
                s_bias_settle = BIAS_SETTLE_MS;
                hal_encoder_reset();
                s_bias_enc_start = 0;
                apply_voltage(s_bias_mV, vCmdL_mV, vCmdR_mV);
                return true;
            } else {
                // Both done
                s_bias_result_R = s_bias_mV;
                s_mode = L2B_IDLE;
                *vCmdL_mV = 0; *vCmdR_mV = 0;

                // Enqueue result RSP via comms — use special mechanism
                // Actually, the result is stored and will be fetched by
                // a completion check. See lab2b_get_bias_result().
                return false;
            }
        }

        // Not moving yet — increase voltage
        s_bias_mV += BIAS_STEP_MV;
        if (s_bias_mV > BIAS_MAX_MV) {
            // Give up — motor might be disconnected
            if (s_bias_phase == 0) {
                s_bias_result_L = -1;  // error marker
                s_bias_phase = 1;
                s_select = 2;
                s_bias_mV = BIAS_START_MV;
                s_bias_settle = BIAS_SETTLE_MS;
                hal_encoder_reset();
                s_bias_enc_start = 0;
                apply_voltage(s_bias_mV, vCmdL_mV, vCmdR_mV);
                return true;
            } else {
                s_bias_result_R = -1;
                s_mode = L2B_IDLE;
                *vCmdL_mV = 0; *vCmdR_mV = 0;
                return false;
            }
        }

        s_bias_settle = BIAS_SETTLE_MS;
        s_bias_enc_start = (s_bias_phase == 0) ?
            hal_encoder_count_L() : hal_encoder_count_R();
        apply_voltage(s_bias_mV, vCmdL_mV, vCmdR_mV);
        return true;
    }

    default:
        *vCmdL_mV = 0; *vCmdR_mV = 0;
        return false;
    }
}

void lab2b_stop(void) {
    s_mode = L2B_IDLE;
}

bool lab2b_is_running(void) {
    return s_mode != L2B_IDLE;
}

// ── Result accessor for FIND_BIAS (called from control_task after completion) ──
void lab2b_get_bias_result(int16_t* biasL, int16_t* biasR) {
    *biasL = s_bias_result_L;
    *biasR = s_bias_result_R;
}
