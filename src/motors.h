// ============================================================================
//  motors.h - TB6612FNG dual N20 motor control
// ============================================================================
#pragma once
#include <stdint.h>

namespace Motors {

// Initialise LEDC PWM, direction pins and bring the driver out of standby.
void begin();

// Drive primitives. `speed` is 0..255 (magnitude); direction is implied by
// the function. Speeds are clamped to the active speed cap (torque mode).
void forward(int speed);
void reverse(int speed);
void turnLeft(int speed);   // pivot in place, nose to the left
void turnRight(int speed);  // pivot in place, nose to the right

// Differential drive for steering toward a bearing. left/right in -255..255.
void drive(int leftSpeed, int rightSpeed);

void stop();   // coast: PWM 0, then STBY LOW (low-power, no holding torque)
void brake();  // active short brake on both motors (STBY stays HIGH)

// "Soft" upper bound applied to every speed magnitude. Re-set every FSM tick
// by Strategy from the torque-mode DIP switch.
void setSpeedCap(int cap);

// "Hard" upper bound — set once (typically by a test or safety wrapper) and
// applied as an additional ceiling on top of the soft cap. The effective
// clamp is min(softCap, hardCap). Default = 255 (no extra limit).
void setHardCap(int cap);

} // namespace Motors
