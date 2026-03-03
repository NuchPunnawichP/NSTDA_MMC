// ═══════════════════════════════════════════════════════════════════════════════
//  lab2a_enc_cal.cpp — Lab 2a: Encoder Calibration Implementation
// ═══════════════════════════════════════════════════════════════════════════════
#include "lab2a_enc_cal.h"
#include "config.h"
#include "hal_encoder.h"
#include <string.h>

static inline void wr_i32(uint8_t* p, int32_t v) {
    p[0] = (uint8_t)(v);       p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}

bool lab2a_handle_test(const CommandRequest* cmd, CommandResult* result) {
    uint8_t test_id = cmd->payload[0];

    memset(result, 0, sizeof(*result));
    result->opcode = cmd->opcode;
    result->req_id = cmd->req_id;
    result->status = RSP_OK;

    switch (test_id) {

    case TEST_ENC_MONITOR: {
        // Return absolute encoder counts (int32 × 2 = 8 bytes)
        int32_t cL = hal_encoder_count_L();
        int32_t cR = hal_encoder_count_R();
        wr_i32(&result->rsp_payload[0], cL);
        wr_i32(&result->rsp_payload[4], cR);
        result->rsp_payload_len = 8;
        return true;
    }

    case TEST_ENC_RESET:
        hal_encoder_reset();
        return true;

    default:
        return false;
    }
}
