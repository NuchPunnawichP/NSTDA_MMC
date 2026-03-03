// ═══════════════════════════════════════════════════════════════════════════════
//  hal_wallsensor.cpp — Wall Sensor Manager Implementation
//
//  Handles two sensor types through unified interface:
//    IR analog (SHARP GP2Y0A51SK0F): ADC → voltage → distance via formula
//    VL6180X (ToF): I2C ranging → mm directly
//
//  VL6180X driver: minimal register-level implementation
//    (no external library, educational & self-contained)
// ═══════════════════════════════════════════════════════════════════════════════
#include "hal_wallsensor.h"
#include "config.h"
#include <Arduino.h>
#include <Wire.h>

// ═══════════════════════════════════════════════════════════════════════════════
// VL6180X Register Addresses (subset for ranging)
// ═══════════════════════════════════════════════════════════════════════════════
#define VL_REG_MODEL_ID                 0x0000
#define VL_REG_SYSTEM_FRESH_RESET       0x0016
#define VL_REG_SYSTEM_INTR_CONFIG       0x0014
#define VL_REG_SYSTEM_INTR_CLEAR        0x0015
#define VL_REG_SYSRANGE_START           0x0018
#define VL_REG_SYSRANGE_INTERMEASURE    0x001B
#define VL_REG_SYSRANGE_THRESH_HIGH     0x0019
#define VL_REG_SYSRANGE_THRESH_LOW      0x001A
#define VL_REG_SYSRANGE_MAX_CONVERGE    0x001C
#define VL_REG_SYSRANGE_EARLY_CONVERGE  0x0022
#define VL_REG_RESULT_RANGE_STATUS      0x004D
#define VL_REG_RESULT_RANGE_VAL         0x0062
#define VL_REG_RESULT_INTR_STATUS       0x004F
#define VL_REG_I2C_SLAVE_ADDR           0x0212
#define VL_MODEL_ID_EXPECTED            0xB4

// ── Per-slot state ──────────────────────────────────────────────────────────
typedef struct {
    uint8_t  type;          // 0=none, 1=IR, 2=VL6180X
    uint8_t  i2c_addr;      // VL6180X I2C address (after re-addressing)
    int8_t   xshut_pin;     // VL6180X XSHUT pin (-1 if unused)
    int8_t   adc_pin;       // IR analog pin (-1 if unused)
    int16_t  offset_mm;     // calibration offset (mm)
    uint16_t last_mm;       // last valid reading (mm)
    bool     online;        // sensor responded OK
} SensorSlot;

static SensorSlot s_slots[WALL_POS_COUNT];
static bool       s_init = false;
static uint8_t    s_active = 0;

// ── I2C Bus 1 reference ─────────────────────────────────────────────────────
// Use Wire1 for VL6180X (separate from IMU on Wire/Bus 0)
// If I2C_BUS1_ENABLED == 0 and we're using IR only, Wire1 is not initialized

// ═══════════════════════════════════════════════════════════════════════════════
// VL6180X I2C Helpers (16-bit register addressing)
// ═══════════════════════════════════════════════════════════════════════════════

static void vl_write8(uint8_t addr, uint16_t reg, uint8_t val) {
    Wire1.beginTransmission(addr);
    Wire1.write((reg >> 8) & 0xFF);   // MSB of register address
    Wire1.write(reg & 0xFF);          // LSB of register address
    Wire1.write(val);
    Wire1.endTransmission();
}

static uint8_t vl_read8(uint8_t addr, uint16_t reg) {
    Wire1.beginTransmission(addr);
    Wire1.write((reg >> 8) & 0xFF);
    Wire1.write(reg & 0xFF);
    Wire1.endTransmission(false);
    Wire1.requestFrom(addr, (uint8_t)1);
    return Wire1.available() ? Wire1.read() : 0;
}

// ── VL6180X: apply recommended tuning (AN4545 SR03 settings) ────────────────
static void vl_load_settings(uint8_t addr) {
    // Mandatory : private registers
    vl_write8(addr, 0x0207, 0x01);
    vl_write8(addr, 0x0208, 0x01);
    vl_write8(addr, 0x0096, 0x00);
    vl_write8(addr, 0x0097, 0xFD);
    vl_write8(addr, 0x00E3, 0x00);
    vl_write8(addr, 0x00E4, 0x04);
    vl_write8(addr, 0x00E5, 0x02);
    vl_write8(addr, 0x00E6, 0x01);
    vl_write8(addr, 0x00E7, 0x03);
    vl_write8(addr, 0x00F5, 0x02);
    vl_write8(addr, 0x00D9, 0x05);
    vl_write8(addr, 0x00DB, 0xCE);
    vl_write8(addr, 0x00DC, 0x03);
    vl_write8(addr, 0x00DD, 0xF8);
    vl_write8(addr, 0x009F, 0x00);
    vl_write8(addr, 0x00A3, 0x3C);
    vl_write8(addr, 0x00B7, 0x00);
    vl_write8(addr, 0x00BB, 0x3C);
    vl_write8(addr, 0x00B2, 0x09);
    vl_write8(addr, 0x00CA, 0x09);
    vl_write8(addr, 0x0198, 0x01);
    vl_write8(addr, 0x01B0, 0x17);
    vl_write8(addr, 0x01AD, 0x00);
    vl_write8(addr, 0x00FF, 0x05);
    vl_write8(addr, 0x0100, 0x05);
    vl_write8(addr, 0x0199, 0x05);
    vl_write8(addr, 0x01A6, 0x1B);
    vl_write8(addr, 0x01AC, 0x3E);
    vl_write8(addr, 0x01A7, 0x1F);
    vl_write8(addr, 0x0030, 0x00);

    // Recommended public registers
    vl_write8(addr, VL_REG_SYSTEM_INTR_CONFIG, 0x24);   // range interrupt on new sample ready
    vl_write8(addr, VL_REG_SYSRANGE_MAX_CONVERGE, 0x31); // max convergence time
    vl_write8(addr, VL_REG_SYSRANGE_INTERMEASURE, 0x09); // 100ms inter-measurement
    vl_write8(addr, 0x003F, 0x46);   // ALS integration time
    vl_write8(addr, 0x0031, 0xFF);   // Calibration
    vl_write8(addr, 0x0040, 0x63);
    vl_write8(addr, 0x002E, 0x01);   // averaging sample period
    vl_write8(addr, VL_REG_SYSRANGE_EARLY_CONVERGE, 0x32); // early convergence

    // Clear fresh-out-of-reset flag
    vl_write8(addr, VL_REG_SYSTEM_FRESH_RESET, 0x00);
}

// ── VL6180X: change I2C address ─────────────────────────────────────────────
static void vl_set_address(uint8_t current_addr, uint8_t new_addr) {
    vl_write8(current_addr, VL_REG_I2C_SLAVE_ADDR, new_addr >> 1);
    // Note: VL6180X stores 7-bit address (shifted right by 1)
}

// ── VL6180X: single-shot range read (mm) ────────────────────────────────────
static uint16_t vl_read_range(uint8_t addr) {
    // Wait for device ready
    uint8_t status = vl_read8(addr, VL_REG_RESULT_RANGE_STATUS);
    if ((status & 0x01) == 0) {
        // Device not ready — return last value or 0
        return 0;
    }

    // Start single-shot measurement
    vl_write8(addr, VL_REG_SYSRANGE_START, 0x01);

    // Wait for measurement complete (with timeout)
    uint32_t start = millis();
    while (millis() - start < 50) {  // 50ms timeout
        uint8_t intr = vl_read8(addr, VL_REG_RESULT_INTR_STATUS);
        if (intr & 0x04) break;  // range complete
    }

    // Read result
    uint8_t range = vl_read8(addr, VL_REG_RESULT_RANGE_VAL);

    // Clear interrupt
    vl_write8(addr, VL_REG_SYSTEM_INTR_CLEAR, 0x07);

    return (uint16_t)range;
}

// ═══════════════════════════════════════════════════════════════════════════════
// IR Analog Sensor Helpers
// ═══════════════════════════════════════════════════════════════════════════════

// GP2Y0A51SK0F: analog output → distance conversion
// Approximate formula: distance_mm = K / (ADC_value - offset)
// Students should calibrate K and offset for their specific sensor
// Default: using a simple inverse model that works for 20-150mm range
#define IR_K_FACTOR             50000   // calibration constant (adjust per sensor)
#define IR_ADC_OFFSET           100     // ADC baseline offset (adjust per sensor)

static uint16_t ir_adc_to_mm(uint16_t adc_raw) {
    // Guard division by zero
    if (adc_raw <= IR_ADC_OFFSET + 10) return IR_MAX_VALID_MM;

    int32_t diff = (int32_t)adc_raw - IR_ADC_OFFSET;
    uint32_t mm = IR_K_FACTOR / (uint32_t)diff;

    // Clamp to valid range
    if (mm < IR_MIN_VALID_MM)  return IR_MIN_VALID_MM;
    if (mm > IR_MAX_VALID_MM)  return IR_MAX_VALID_MM;
    return (uint16_t)mm;
}

// ═══════════════════════════════════════════════════════════════════════════════
// Sensor Manager: Slot Configuration Tables
// ═══════════════════════════════════════════════════════════════════════════════

// VL6180X XSHUT pins per position
static const int8_t VL_XSHUT_PINS[WALL_POS_COUNT] = {
    PIN_VL_XSHUT_LEFT, PIN_VL_XSHUT_FRONT_L,
    PIN_VL_XSHUT_FRONT_R, PIN_VL_XSHUT_RIGHT
};

// VL6180X target I2C addresses per position
static const uint8_t VL_ADDRS[WALL_POS_COUNT] = {
    VL6180X_ADDR_LEFT, VL6180X_ADDR_FRONT_L,
    VL6180X_ADDR_FRONT_R, VL6180X_ADDR_RIGHT
};

// IR analog pins per position
static const int8_t IR_PINS[WALL_POS_COUNT] = {
    PIN_IR_LEFT, PIN_IR_FRONT_L,
    PIN_IR_FRONT_R, PIN_IR_RIGHT
};

// Slot types per position (from config.h)
static const uint8_t SLOT_TYPES[WALL_POS_COUNT] = {
    SLOT_TYPE_LEFT, SLOT_TYPE_FRONT_L,
    SLOT_TYPE_FRONT_R, SLOT_TYPE_RIGHT
};

// Offsets per position
static const int16_t SLOT_OFFSETS[WALL_POS_COUNT] = {
    WALL_OFFSET_LEFT_MM, WALL_OFFSET_FRONT_L_MM,
    WALL_OFFSET_FRONT_R_MM, WALL_OFFSET_RIGHT_MM
};

// ═══════════════════════════════════════════════════════════════════════════════
// Public API
// ═══════════════════════════════════════════════════════════════════════════════

bool hal_wall_init(void) {
    s_active = 0;

    // ── Initialize VL6180X sensors if needed ──
    bool need_i2c1 = false;
    for (int i = 0; i < WALL_POS_COUNT; i++) {
        uint8_t t;
        if (WALL_SENSOR_TYPE == 3) {
            t = SLOT_TYPES[i];  // Mixed: per-slot type
        } else {
            t = WALL_SENSOR_TYPE;  // Uniform type
        }
        s_slots[i].type      = t;
        s_slots[i].i2c_addr  = VL_ADDRS[i];
        s_slots[i].xshut_pin = VL_XSHUT_PINS[i];
        s_slots[i].adc_pin   = IR_PINS[i];
        s_slots[i].offset_mm = SLOT_OFFSETS[i];
        s_slots[i].last_mm   = 0;
        s_slots[i].online    = false;

        if (t == WALL_SNS_VL6180X) need_i2c1 = true;
    }

    // ── Start I2C Bus 1 if any VL6180X sensors ──
#if I2C_BUS1_ENABLED
    if (need_i2c1) {
        Wire1.begin(PIN_I2C1_SDA, PIN_I2C1_SCL, I2C1_FREQ);
        delay(10);

        // ── VL6180X re-addressing sequence ──
        // 1. Hold all XSHUT LOW (power off all sensors)
        for (int i = 0; i < WALL_POS_COUNT; i++) {
            if (s_slots[i].type == WALL_SNS_VL6180X && s_slots[i].xshut_pin >= 0) {
                pinMode(s_slots[i].xshut_pin, OUTPUT);
                digitalWrite(s_slots[i].xshut_pin, LOW);
            }
        }
        delay(10);

        // 2. Enable one at a time, re-address each
        for (int i = 0; i < WALL_POS_COUNT; i++) {
            if (s_slots[i].type != WALL_SNS_VL6180X) continue;
            if (s_slots[i].xshut_pin < 0) {
                // No XSHUT control — try default address (only works for 1 sensor)
                uint8_t id = vl_read8(VL6180X_ADDR_DEFAULT, VL_REG_MODEL_ID);
                if (id == VL_MODEL_ID_EXPECTED) {
                    vl_load_settings(VL6180X_ADDR_DEFAULT);
                    s_slots[i].i2c_addr = VL6180X_ADDR_DEFAULT;
                    s_slots[i].online   = true;
                    s_active++;
                }
                continue;
            }

            // Release XSHUT → sensor boots with default address
            digitalWrite(s_slots[i].xshut_pin, HIGH);
            delay(10);  // VL6180X boot time ~1.3ms, use 10ms for safety

            // Verify it's there
            uint8_t id = vl_read8(VL6180X_ADDR_DEFAULT, VL_REG_MODEL_ID);
            if (id != VL_MODEL_ID_EXPECTED) continue;  // sensor not responding

            // Change to unique address
            vl_set_address(VL6180X_ADDR_DEFAULT, VL_ADDRS[i]);
            delay(5);

            // Verify new address
            id = vl_read8(VL_ADDRS[i], VL_REG_MODEL_ID);
            if (id != VL_MODEL_ID_EXPECTED) continue;

            // Load tuning settings
            vl_load_settings(VL_ADDRS[i]);
            s_slots[i].online = true;
            s_active++;
        }
    }
#endif

    // ── Initialize IR analog sensors ──
    for (int i = 0; i < WALL_POS_COUNT; i++) {
        if (s_slots[i].type == WALL_SNS_IR_ANALOG && s_slots[i].adc_pin >= 0) {
            analogReadResolution(IR_ADC_BITS);
            pinMode(s_slots[i].adc_pin, INPUT);
            s_slots[i].online = true;
            s_active++;
        }
    }

    s_init = (s_active > 0);
    return s_init;
}

void hal_wall_update(void) {
    if (!s_init) return;

    for (int i = 0; i < WALL_POS_COUNT; i++) {
        if (!s_slots[i].online) continue;

        uint16_t raw_mm = 0;

        if (s_slots[i].type == WALL_SNS_VL6180X) {
#if I2C_BUS1_ENABLED
            raw_mm = vl_read_range(s_slots[i].i2c_addr);
            // Clamp to valid range
            if (raw_mm < VL6180X_RANGE_MIN_MM) raw_mm = VL6180X_RANGE_MIN_MM;
            if (raw_mm > VL6180X_RANGE_MAX_MM) raw_mm = VL6180X_RANGE_MAX_MM;
#endif
        }
        else if (s_slots[i].type == WALL_SNS_IR_ANALOG) {
            uint16_t adc = analogRead(s_slots[i].adc_pin);
            raw_mm = ir_adc_to_mm(adc);
        }

        // Apply offset
        int16_t adjusted = (int16_t)raw_mm + s_slots[i].offset_mm;
        if (adjusted < 0) adjusted = 0;

        s_slots[i].last_mm = (uint16_t)adjusted;
    }
}

uint16_t hal_wall_get_mm(uint8_t position) {
    if (position >= WALL_POS_COUNT) return 0;
    return s_slots[position].last_mm;
}

void hal_wall_get_all(uint16_t out[WALL_POS_COUNT]) {
    for (int i = 0; i < WALL_POS_COUNT; i++) {
        out[i] = s_slots[i].last_mm;
    }
}

bool hal_wall_has_left(void) {
    return s_slots[SNS_POS_LEFT].last_mm > 0 &&
           s_slots[SNS_POS_LEFT].last_mm < WALL_THRESHOLD_SIDE_MM;
}

bool hal_wall_has_front_left(void) {
    return s_slots[SNS_POS_FRONT_LEFT].last_mm > 0 &&
           s_slots[SNS_POS_FRONT_LEFT].last_mm < WALL_THRESHOLD_FRONT_MM;
}

bool hal_wall_has_front_right(void) {
    return s_slots[SNS_POS_FRONT_RIGHT].last_mm > 0 &&
           s_slots[SNS_POS_FRONT_RIGHT].last_mm < WALL_THRESHOLD_FRONT_MM;
}

bool hal_wall_has_right(void) {
    return s_slots[SNS_POS_RIGHT].last_mm > 0 &&
           s_slots[SNS_POS_RIGHT].last_mm < WALL_THRESHOLD_SIDE_MM;
}

bool hal_wall_has_front(void) {
    return hal_wall_has_front_left() || hal_wall_has_front_right();
}

uint8_t hal_wall_bitmap(void) {
    uint8_t bm = 0;
    if (hal_wall_has_left())        bm |= (1 << 0);
    if (hal_wall_has_front_left())  bm |= (1 << 1);
    if (hal_wall_has_front_right()) bm |= (1 << 2);
    if (hal_wall_has_right())       bm |= (1 << 3);
    return bm;
}

uint8_t hal_wall_get_type(uint8_t position) {
    if (position >= WALL_POS_COUNT) return 0;
    return s_slots[position].type;
}

bool    hal_wall_is_init(void)      { return s_init; }
uint8_t hal_wall_active_count(void) { return s_active; }

// ── Calibration ─────────────────────────────────────────────────────────────
static uint16_t s_cal_raw[WALL_POS_COUNT] = {0,0,0,0};  // last cal raw averages

void hal_wall_calibrate(uint16_t samples) {
    // Measure-only: read N samples, store raw averages (before offset).
    // Does NOT change offsets — use hal_wall_set_offset() to apply.
    if (!s_init || samples == 0) return;

    uint32_t sums[WALL_POS_COUNT] = {0,0,0,0};
    uint16_t counts[WALL_POS_COUNT] = {0,0,0,0};

    for (uint16_t i = 0; i < samples; i++) {
        hal_wall_update();
        for (int p = 0; p < WALL_POS_COUNT; p++) {
            if (!s_slots[p].online) continue;
            // Read raw (before offset) by subtracting current offset
            int16_t raw = (int16_t)s_slots[p].last_mm - s_slots[p].offset_mm;
            if (raw > 0) {
                sums[p] += raw;
                counts[p]++;
            }
        }
        delay(25);  // ~40Hz sensor update rate
    }

    for (int p = 0; p < WALL_POS_COUNT; p++) {
        if (counts[p] > 0) {
            s_cal_raw[p] = (uint16_t)(sums[p] / counts[p]);
        }
    }
}

void hal_wall_set_offset(uint8_t position, int16_t offset_mm) {
    if (position < WALL_POS_COUNT) {
        s_slots[position].offset_mm = offset_mm;
    }
}

int16_t hal_wall_get_offset(uint8_t position) {
    if (position < WALL_POS_COUNT) {
        return s_slots[position].offset_mm;
    }
    return 0;
}

uint16_t hal_wall_get_cal_raw(uint8_t position) {
    if (position < WALL_POS_COUNT) {
        return s_cal_raw[position];
    }
    return 0;
}
