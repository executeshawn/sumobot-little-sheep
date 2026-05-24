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

// Aggressive mode (SW1 ON) shortens detection range so the bot only commits
// to very close targets but does so at higher speed.
constexpr uint16_t TOF_THRESH_AGGRESSIVE_DELTA = 150; // subtracted from above

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
//  Speed profiles (0..255). "torque" mode (SW3 ON) caps top speed.
// ---------------------------------------------------------------------------
constexpr int SPEED_FULL   = 255; // max
constexpr int SPEED_ATTACK = 230; // attack drive
constexpr int SPEED_SEARCH = 140; // search rotation/sweep
constexpr int SPEED_TURN   = 180; // evade/turn pivots
constexpr int SPEED_TORQUE_CAP = 160; // SW3 ON: clamp all speeds to this

// Aggressive mode (SW1 ON) attack speed override
constexpr int SPEED_ATTACK_AGGRESSIVE = 255;

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
//  Edge detection must win the race against driving off the ring, so it is
//  polled fastest and checked first in the FSM. The front ToF group runs at a
//  medium rate (engagement-critical); the side/rear group runs slow (only used
//  to break ties / detect flanking).
// ---------------------------------------------------------------------------
constexpr uint32_t FSM_UPDATE_MS     = 5;   // FSM tick (consumes cached data)
constexpr uint32_t EDGE_POLL_MS      = 8;   // FAST   : QTR edge read + cache
constexpr uint32_t TOF_FRONT_POLL_MS = 30;  // MEDIUM : harvest front 3 ToF
constexpr uint32_t TOF_SIDE_POLL_MS  = 90;  // SLOW   : harvest side/rear 3 ToF

// VL53L0X continuous-ranging inter-measurement periods (ms). The chip ranges
// on its own; harvesting above just reads the latest completed result without
// blocking. Front ranges faster than side/rear.
constexpr uint16_t TOF_CONT_FRONT_MS = 25;
constexpr uint16_t TOF_CONT_SIDE_MS  = 70;

// Search behavior
constexpr uint32_t SWEEP_SEGMENT_MS = 600;  // time per sweep direction (SW2 OFF)

// ---------------------------------------------------------------------------
//  LED blink periods (ms half-period)
// ---------------------------------------------------------------------------
constexpr uint32_t LED_BLINK_SLOW = 500; // IDLE
constexpr uint32_t LED_BLINK_FAST = 100; // CALIBRATE
constexpr uint32_t LED_BLINK_RAPID = 60; // ATTACK
