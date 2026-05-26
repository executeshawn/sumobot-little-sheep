// ============================================================================
//  movement.h - Movement Calibration Layer (open-loop trim + pivot table)
// ============================================================================
//
//  This module sits between Strategy and Motors::. It does NOT change the
//  validated polarity contract; it only adds two open-loop adjustments:
//
//    1. Per-channel PWM trim (MOVE_TRIM_LEFT / MOVE_TRIM_RIGHT in config.h)
//       to compensate for mechanical mismatch between the two TT motors.
//    2. An empirical pivot-by-degree helper that converts an angle into a
//       timed Motors::turnLeft/Right call using bench-measured ms-per-deg
//       constants.
//
//  Closed-loop drift correction and wall alignment were intentionally NOT
//  implemented: this build has no encoders, no IMU, and the VL53L0X ring is
//  configured as an outward-facing opponent detector with no fixed
//  reference surface. Any "VL53 lateral steering" or "wall align" routine
//  here would have no ground truth to act on.
//
//  Not wired into the FSM yet. Strategy continues to call Motors:: directly
//  until trim/pivot constants are bench-measured and validated.
//
#pragma once
#include <stdint.h>

namespace Movement {

// Drive primitives. Signed speed -255..255 for drive(); the named helpers
// take magnitudes 0..255 (direction is implied by the function). Trim is
// applied internally; callers pass the unmodified target speed.
void forward(int speed);
void reverse(int speed);
void pivotLeft(int speed);
void pivotRight(int speed);
void drive(int leftSpeed, int rightSpeed);
void stop();

// Pivot by a target angle in degrees using the bench-measured
// MOVE_MS_PER_DEG_* constants. BLOCKING for the computed duration, then
// commands Motors::stop(). FSM must only call these from states that
// tolerate a blocking pivot (i.e. not from the main combat loop).
void pivotLeftDeg (int degrees, int speed);
void pivotRightDeg(int degrees, int speed);

} // namespace Movement
