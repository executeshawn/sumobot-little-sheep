// ============================================================================
//  sensors_tof.cpp - 6x VL53L0X bring-up and opponent bearing
//                    Pololu VL53L0X library (pololu/VL53L0X@^1.3.1)
// ============================================================================
#include "sensors_tof.h"
#include "pins.h"
#include "config.h"
#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>

namespace {

// Per-sensor static config, indexed by TofIndex.
const int      XSHUT_PIN[TOF_N] = {
    PIN_XSHUT_FL, PIN_XSHUT_FC, PIN_XSHUT_FR,
    PIN_XSHUT_L,  PIN_XSHUT_R,  PIN_XSHUT_RR
};
// Unique I2C addresses assigned during bring-up (all distinct, != default 0x29).
const uint8_t  I2C_ADDR[TOF_N] = { 0x30, 0x31, 0x32, 0x33, 0x34, 0x35 };

const uint16_t THRESH[TOF_N] = {
    TOF_THRESH_FRONT_LEFT, TOF_THRESH_FRONT_CENTER, TOF_THRESH_FRONT_RIGHT,
    TOF_THRESH_LEFT,       TOF_THRESH_RIGHT,         TOF_THRESH_REAR
};

// Front group = indices 0..2 (FL/FC/FR); side/rear group = indices 3..5.
constexpr int FRONT_BEGIN = TOF_FL, FRONT_END = TOF_L;   // [0,3)
constexpr int SIDE_BEGIN  = TOF_L,  SIDE_END  = TOF_N;    // [3,6)

// Per-sensor read timeout (ms). Caps how long a single readRangeContinuous
// call can stall the poll if the sensor stops responding.
constexpr uint16_t TOF_READ_TIMEOUT_MS = 50;

// Measurement timing budget (microseconds). 20 ms approximates the
// "high speed" preset that the previous library exposed.
constexpr uint32_t TOF_TIMING_BUDGET_US = 20000;

VL53L0X  g_lox[TOF_N];
bool     g_online[TOF_N] = { false, false, false, false, false, false };
uint16_t g_cache[TOF_N]  = {
    TOF_OUT_OF_RANGE, TOF_OUT_OF_RANGE, TOF_OUT_OF_RANGE,
    TOF_OUT_OF_RANGE, TOF_OUT_OF_RANGE, TOF_OUT_OF_RANGE
};

// Read one sensor and update its cache slot. The Pololu driver does not
// expose a non-blocking "is result ready" hook for continuous mode, so this
// call may briefly block up to TOF_READ_TIMEOUT_MS while waiting for the
// next measurement; on timeout the slot is marked TOF_OUT_OF_RANGE.
void harvest(int i) {
    if (!g_online[i]) return;
    uint16_t mm = g_lox[i].readRangeContinuousMillimeters();
    if (g_lox[i].timeoutOccurred()) {
        g_cache[i] = TOF_OUT_OF_RANGE;
    } else {
        g_cache[i] = mm;
    }
}

} // namespace

namespace ToF {

int begin() {
    // 1) Hold every sensor in reset (XSHUT LOW). Each sensor now sits at its
    //    power-on default address 0x29 but is disabled.
    for (int i = 0; i < TOF_N; ++i) {
        pinMode(XSHUT_PIN[i], OUTPUT);
        digitalWrite(XSHUT_PIN[i], LOW);
    }
    delay(20); // ensure all are fully shut down before we start

    int count = 0;
    // 2) Wake exactly one sensor at a time, then move it off 0x29 so the next
    //    one (which also wakes at 0x29) does not collide.
    for (int i = 0; i < TOF_N; ++i) {
        digitalWrite(XSHUT_PIN[i], HIGH);   // wake sensor i (boots at 0x29)
        delay(XSHUT_BOOT_DELAY_MS);          // let the chip boot

        g_lox[i].setBus(&Wire);
        g_lox[i].setTimeout(TOF_READ_TIMEOUT_MS);

        // init() talks to the chip at the current address (0x29 right now).
        if (g_lox[i].init()) {
            // 3) Move this sensor off the default address before waking the
            //    next one. Subsequent calls use I2C_ADDR[i].
            g_lox[i].setAddress(I2C_ADDR[i]);
            g_lox[i].setMeasurementTimingBudget(TOF_TIMING_BUDGET_US);

            // 4) Start free-running continuous ranging. Front group runs at
            //    a shorter inter-measurement period than the side/rear group.
            const uint32_t period = (i < FRONT_END) ? TOF_CONT_FRONT_MS
                                                    : TOF_CONT_SIDE_MS;
            g_lox[i].startContinuous(period);

            g_online[i] = true;
            ++count;
        } else {
            g_online[i] = false;
            Serial.printf("[ToF] sensor %d FAILED to init (would be 0x%02X)\n",
                          i, I2C_ADDR[i]);
        }
        // Sensor i now answers at I2C_ADDR[i]; leave XSHUT HIGH and continue.
    }
    return count;
}

void pollFront() {
    for (int i = FRONT_BEGIN; i < FRONT_END; ++i) harvest(i);
}

void pollSide() {
    for (int i = SIDE_BEGIN; i < SIDE_END; ++i) harvest(i);
}

TofReadings readAllSensors() {
    TofReadings r;
    for (int i = 0; i < TOF_N; ++i) {
        r.online[i] = g_online[i];
        r.mm[i]     = g_online[i] ? g_cache[i] : TOF_OUT_OF_RANGE;
    }
    return r;
}

OpponentDir detectOpponent(const TofReadings &r) {
    uint16_t best = TOF_OUT_OF_RANGE;
    int      bestIdx = -1;

    for (int i = 0; i < TOF_N; ++i) {
        if (!r.online[i]) continue;
        if (r.mm[i] <= THRESH[i] && r.mm[i] < best) {
            best = r.mm[i];
            bestIdx = i;
        }
    }

    switch (bestIdx) {
        case TOF_FL: return OpponentDir::FRONT_LEFT;
        case TOF_FC: return OpponentDir::FRONT_CENTER;
        case TOF_FR: return OpponentDir::FRONT_RIGHT;
        case TOF_L:  return OpponentDir::LEFT;
        case TOF_R:  return OpponentDir::RIGHT;
        case TOF_RR: return OpponentDir::REAR;
        default:     return OpponentDir::NONE;
    }
}

const char *dirName(OpponentDir d) {
    switch (d) {
        case OpponentDir::FRONT_LEFT:   return "FRONT_LEFT";
        case OpponentDir::FRONT_CENTER: return "FRONT_CENTER";
        case OpponentDir::FRONT_RIGHT:  return "FRONT_RIGHT";
        case OpponentDir::LEFT:         return "LEFT";
        case OpponentDir::RIGHT:        return "RIGHT";
        case OpponentDir::REAR:         return "REAR";
        default:                        return "NONE";
    }
}

} // namespace ToF
