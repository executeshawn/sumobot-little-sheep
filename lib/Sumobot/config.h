// ============================================================================
//  config.h - tunable parameters
// ============================================================================
#pragma once
#include <stdint.h>

// ---------------------------------------------------------------------------
//  ToF opponent-detection thresholds (mm). A reading <= threshold at a given
//  sensor counts as "opponent present" in that direction. 9999 = no target.
//  Front sensors reach further than side/rear for forward engagement.
// ---------------------------------------------------------------------------
constexpr uint16_t TOF_THRESH_FRONT_LEFT   = 400;
constexpr uint16_t TOF_THRESH_FRONT_CENTER = 500;
constexpr uint16_t TOF_THRESH_FRONT_RIGHT  = 400;
constexpr uint16_t TOF_THRESH_LEFT         = 300;
constexpr uint16_t TOF_THRESH_RIGHT        = 300;
constexpr uint16_t TOF_THRESH_REAR         = 250;

constexpr uint16_t TOF_OUT_OF_RANGE = 9999; // sentinel for invalid/timeout

// ---------------------------------------------------------------------------
//  Edge (QTR) detection
//  RC sensors: LOWER calibrated value = MORE reflective = WHITE ring border.
//  Calibrated readings are 0..1000. A value below this threshold = on edge.
// ---------------------------------------------------------------------------
constexpr uint16_t EDGE_THRESHOLD = 250;

// ---------------------------------------------------------------------------
//  Motor PWM (TB6612FNG via ESP32 LEDC)
// ---------------------------------------------------------------------------
constexpr uint32_t MOTOR_PWM_FREQ = 5000; // 5 kHz - quiet, well within TB6612
constexpr uint8_t  MOTOR_PWM_RES  = 8;    // 8-bit -> duty 0..255
constexpr uint8_t  MOTOR_CH_LEFT  = 0;    // LEDC channel for left motor
constexpr uint8_t  MOTOR_CH_RIGHT = 1;    // LEDC channel for right motor

// ---------------------------------------------------------------------------
//  Speed profiles (0..255). Deterministic - no DIP-switch overrides.
// ---------------------------------------------------------------------------
constexpr int SPEED_FULL   = 255; // max
constexpr int SPEED_ATTACK = 230; // attack drive
constexpr int SPEED_SEARCH = 140; // search rotation/sweep
constexpr int SPEED_TURN   = 180; // evade/turn pivots

// ---------------------------------------------------------------------------
//  Timing
// ---------------------------------------------------------------------------
constexpr uint32_t START_DELAY_MS      = 5000; // mandatory pre-match countdown
constexpr uint32_t ATTACK_WATCHDOG_MS  = 10000;// force SEARCH if stuck attacking
constexpr uint32_t EVADE_REVERSE_MS    = 350;  // reverse-off-edge duration
constexpr uint32_t EVADE_TURN_MS       = 450;  // ~180 deg pivot duration
constexpr uint32_t XSHUT_BOOT_DELAY_MS = 10;   // settle after raising XSHUT

// ---------------------------------------------------------------------------
//  Multi-rate sensing (edge has top priority -> shortest period)
// ---------------------------------------------------------------------------
constexpr uint32_t FSM_UPDATE_MS     = 5;   // FSM tick (consumes cached data)
constexpr uint32_t EDGE_POLL_MS      = 8;   // FAST   : QTR edge read + cache
constexpr uint32_t TOF_FRONT_POLL_MS = 30;  // MEDIUM : harvest front 3 ToF
constexpr uint32_t TOF_SIDE_POLL_MS  = 90;  // SLOW   : harvest side/rear 3 ToF

// VL53L0X continuous-ranging inter-measurement periods (ms).
constexpr uint16_t TOF_CONT_FRONT_MS = 25;
constexpr uint16_t TOF_CONT_SIDE_MS  = 70;

// Search behaviour: deterministic alternating sweep.
constexpr uint32_t SWEEP_SEGMENT_MS = 600;

// ---------------------------------------------------------------------------
//  Movement calibration (open-loop, no sensor feedback).
//
//  Why open-loop only: this build has no encoders, no IMU, and the VL53L0X
//  side sensors look outward at the opponent / open arena (not at walls or
//  ground), so there is no reference surface to close a loop against.
//
//  TRIM: per-channel PWM scaler applied in Movement:: above Motors::. 1.00 =
//  no change. Lower the *faster* side to straighten a forward drift. Range
//  clamped to [0.50, 1.00] in code so trim only ever *slows* a motor (never
//  asks for more PWM than the caller requested).
//
//  MS_PER_DEG: empirical bench constant for pivot-by-angle helpers. Measure
//  by commanding a 360-deg pivot at SPEED_TURN and timing it; divide by 360.
//  Defaults are placeholders until measured.
// ---------------------------------------------------------------------------
constexpr float MOVE_TRIM_LEFT     = 1.00f;
constexpr float MOVE_TRIM_RIGHT    = 1.00f;
constexpr float MOVE_MS_PER_DEG_LEFT  = 6.0f;  // placeholder until bench-measured
constexpr float MOVE_MS_PER_DEG_RIGHT = 6.0f;  // placeholder until bench-measured
