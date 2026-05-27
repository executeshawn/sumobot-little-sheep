// ============================================================================
//  movement.cpp - open-loop trim + empirical pivot table over Motors::
// ============================================================================
#include "movement.h"
#include "motors.h"
#include "config.h"
#include <Arduino.h>

namespace {

// Clamp trim into [0.50, 1.00] so the wrapper can only ever *slow* a motor;
// it must never ask Motors:: for more PWM than the caller specified. This
// keeps the speed cap in Motors:: authoritative.
inline float clampTrim(float t) {
    if (t < 0.50f) return 0.50f;
    if (t > 1.00f) return 1.00f;
    return t;
}

// Apply the per-channel trim to a signed speed. Preserves sign; only
// magnitude is scaled. Returns an int in -255..255.
inline int applyTrim(int signedSpeed, float trim) {
    const float scaled = (float)signedSpeed * clampTrim(trim);
    int out = (int)scaled;
    if (out >  255) out =  255;
    if (out < -255) out = -255;
    return out;
}

// Compute pivot duration from the bench constant. Defensive clamps keep a
// stray bad input (e.g. negative degrees or a missing constant) from
// commanding a runaway pivot.
inline uint32_t pivotDurationMs(int degrees, float msPerDeg) {
    if (degrees <= 0)        return 0;
    if (degrees > 360)       degrees = 360;
    if (msPerDeg < 1.0f)     msPerDeg = 1.0f;  // sanity floor
    if (msPerDeg > 30.0f)    msPerDeg = 30.0f; // sanity ceiling
    return (uint32_t)((float)degrees * msPerDeg);
}

} // namespace

namespace Movement {

void drive(int leftSpeed, int rightSpeed) {
    const int l = applyTrim(leftSpeed,  MOVE_TRIM_LEFT);
    const int r = applyTrim(rightSpeed, MOVE_TRIM_RIGHT);
    Motors::drive(l, r);
}

void forward(int speed) {
    speed = speed < 0 ? -speed : speed;
    drive( speed,  speed);
}

void reverse(int speed) {
    speed = speed < 0 ? -speed : speed;
    drive(-speed, -speed);
}

void pivotLeft(int speed) {
    speed = speed < 0 ? -speed : speed;
    // Pivots intentionally bypass trim: trim compensates straight-line
    // drift, which has no analogue when the wheels turn opposite directions.
    Motors::turnLeft(speed);
}

void pivotRight(int speed) {
    speed = speed < 0 ? -speed : speed;
    Motors::turnRight(speed);
}

void stop() {
    Motors::stop();
}

void pivotLeftDeg(int degrees, int speed) {
    const uint32_t ms = pivotDurationMs(degrees, MOVE_MS_PER_DEG_LEFT);
    if (ms == 0) return;
    Motors::turnLeft(speed < 0 ? -speed : speed);
    delay(ms);
    Motors::stop();
}

void pivotRightDeg(int degrees, int speed) {
    const uint32_t ms = pivotDurationMs(degrees, MOVE_MS_PER_DEG_RIGHT);
    if (ms == 0) return;
    Motors::turnRight(speed < 0 ? -speed : speed);
    delay(ms);
    Motors::stop();
}

} // namespace Movement
