// ============================================================================
//  sensors_edge.cpp - QTR-MD-03RC in RC (digital discharge-timing) mode
// ============================================================================
//
//  RC operation: the library charges each sensor's capacitor (pin OUTPUT HIGH),
//  releases it (pin INPUT) and times the discharge. A reflective WHITE surface
//  discharges fast -> LOW timing value. The black mat -> HIGH timing value.
//  After calibration, readCalibrated() maps to 0..1000 (0 = most reflective).
//
#include "sensors_edge.h"
#include "pins.h"
#include "config.h"
#include <Arduino.h>
#include <QTRSensors.h>

namespace {

QTRSensors  g_qtr;
uint16_t    g_values[QTR_COUNT] = { 1000, 1000, 1000 }; // default = black mat

// Index order matches physical layout: 0 = left, 1 = center, 2 = right.
const uint8_t QTR_PINS[QTR_COUNT] = {
    (uint8_t)PIN_QTR_LEFT, (uint8_t)PIN_QTR_CENTER, (uint8_t)PIN_QTR_RIGHT
};

} // namespace

namespace Edge {

void begin() {
    g_qtr.setTypeRC();
    g_qtr.setSensorPins(QTR_PINS, QTR_COUNT);
    // ~2.5 ms max discharge timeout is the QTRSensors default; fine for RC.
}

void calibrateEdge() {
    // One calibration pass = several reads; the caller spins the robot so the
    // array passes over both mat and border between calls.
    g_qtr.calibrate();
}

void poll() {
    g_qtr.readCalibrated(g_values); // 0 (white) .. 1000 (black)
}

bool isOnEdge() {
    for (int i = 0; i < QTR_COUNT; ++i) {
        if (g_values[i] < EDGE_THRESHOLD) return true; // white border seen
    }
    return false;
}

EdgeDir edgeDirection() {
    const bool left   = g_values[0] < EDGE_THRESHOLD;
    const bool center = g_values[1] < EDGE_THRESHOLD;
    const bool right  = g_values[2] < EDGE_THRESHOLD;

    if (left && right)          return EdgeDir::FRONT;
    if (center && !left && !right) return EdgeDir::CENTER;
    if (left)                   return EdgeDir::LEFT;
    if (right)                  return EdgeDir::RIGHT;
    if (center)                 return EdgeDir::CENTER;
    return EdgeDir::NONE;
}

} // namespace Edge
