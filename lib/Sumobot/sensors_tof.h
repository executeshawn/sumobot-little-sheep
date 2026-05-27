// ============================================================================
//  sensors_tof.h - 6x VL53L0X time-of-flight ring (XSHUT address method)
// ============================================================================
#pragma once
#include <stdint.h>

// Fixed sensor index order. Used as array index throughout.
enum TofIndex : uint8_t {
    TOF_FL = 0, // front-left
    TOF_FC,     // front-center
    TOF_FR,     // front-right
    TOF_L,      // left side
    TOF_R,      // right side
    TOF_RR,     // rear
    TOF_N       // count
};

// Direction of a detected opponent (highest-priority single bearing).
enum class OpponentDir : uint8_t {
    NONE,
    FRONT_LEFT,
    FRONT_CENTER,
    FRONT_RIGHT,
    LEFT,
    RIGHT,
    REAR
};

struct TofReadings {
    uint16_t mm[TOF_N];     // distance per sensor, TOF_OUT_OF_RANGE if invalid
    bool     online[TOF_N]; // false if the sensor failed to initialise
};

namespace ToF {

// Runs the full XSHUT bring-up + address-reassignment sequence, then starts
// continuous ranging on every sensor. Returns the number that came online.
int begin();

// Non-blocking harvest of the latest completed continuous measurement for the
// front group (FL/FC/FR). Call from the MEDIUM-rate loop.
void pollFront();

// Non-blocking harvest for the side/rear group (L/R/RR). Call from the SLOW
// loop. Splitting the two lets edge + front sensing stay responsive.
void pollSide();

// Returns the most recently cached readings (updated by pollFront/pollSide).
// Out-of-range / timeout entries are TOF_OUT_OF_RANGE.
TofReadings readAllSensors();

// Reduces a reading set to a single opponent bearing using config thresholds.
OpponentDir detectOpponent(const TofReadings &r);

// Human-readable label for logging.
const char *dirName(OpponentDir d);

} // namespace ToF
