// ============================================================================
//  TEST: Hand Tracker - PID-controlled target following (validation firmware)
// ============================================================================
//
//  Purpose
//    Standalone demonstration firmware for the Feedback & Control Systems
//    requirement:
//
//        "The Hand Tracker - Robot must smoothly follow a target via PID."
//
//    The robot continuously samples the three FRONT VL53L0X sensors
//    (FL / FC / FR), computes a discrete heading error from which sensor
//    sees the nearest target, runs a real PID controller on that error,
//    and applies the controller output as a differential bias on top of a
//    distance-driven translational speed. The result is a smooth turn-and-
//    approach behaviour that holds a fixed standoff from a hand.
//
//  Isolation from production firmware
//    This sketch deliberately does NOT include or call:
//      * the FSM / Strategy module (strategy.h)
//      * the Movement abstraction layer (movement.h)
//      * the edge (QTR) sensor subsystem (sensors_edge.h)
//      * the side / rear ToF sensors (only pollFront() is used)
//      * any tournament thresholds from config.h
//    It only uses Motors:: (the hardware abstraction with the bench-verified
//    polarity contract) and ToF:: for the front 3 sensors. All tuning
//    constants are local to this file so production tuning can evolve
//    independently.
//
//  ===========================  S A F E T Y  ============================
//   * This firmware ignores the edge sensors. NEVER run it free on the
//     dohyo. Either chock the wheels on the bench or tether the robot.
//   * Motors are hard-capped to HAND_TRACKER_HARD_CAP regardless of PID
//     output, so a runaway integrator cannot reach combat speeds.
//   * A 3-second startup grace period prints a countdown before the
//     motor bridge is allowed to drive.
//   * Lost-target watchdog: if no front sensor sees a hand within
//     HAND_DETECT_MAX_MM for LOST_TARGET_TIMEOUT_MS the bot stops and
//     the integrator is zeroed.
//  =======================================================================
//
//  Build / flash:  pio run -e test_hand_tracker -t upload
//  Watch:          pio device monitor -e test_hand_tracker
//
#include <Arduino.h>
#include <Wire.h>
#include "pins.h"
#include "config.h"     // only for TOF_OUT_OF_RANGE sentinel; no tournament tuning consumed
#include "motors.h"
#include "sensors_tof.h"

// ---------------------------------------------------------------------------
//  Tunable constants - local to this firmware. Do NOT move into config.h:
//  these are demo / validation parameters and must remain independent of
//  the tournament tuning that drives the production FSM.
// ---------------------------------------------------------------------------

// --- PID gains (heading) -----------------------------------------------
//  Error is discrete {-1, 0, +1}, so Kp directly scales to PWM counts.
//  Start with Kp only; introduce Kd to damp oscillation; add Ki only if a
//  measurable steady-state offset persists. See tuning guide at the bottom.
constexpr float    PID_KP                  = 35.0f;
constexpr float    PID_KI                  =  0.0f;
constexpr float    PID_KD                  =  8.0f;

// --- Translational base speed ------------------------------------------
//  Forward / reverse speed cap derived from the distance error. The PID
//  heading output is *added* on top of this as a differential bias.
//  Default chosen above measured N20 stiction (~60 PWM) so the wheels
//  actually roll once distance error opens the throttle.
constexpr int      BASE_SPEED              = 90;

// --- Distance control --------------------------------------------------
//  Hand-following standoff. Proportional only - simple and sufficient
//  for a demonstrable hold-position behaviour.
constexpr uint16_t TARGET_DISTANCE_MM      = 200;   // hold this gap to the hand
constexpr float    DISTANCE_KP             = 0.50f; // PWM per mm of error
constexpr uint16_t DISTANCE_DEADBAND_MM    = 25;    // |err| < this -> hold

// --- Target acquisition ------------------------------------------------
//  Any front sensor reading <= this counts as "hand present". Independent
//  of the production TOF_THRESH_FRONT_* values by design.
constexpr uint16_t HAND_DETECT_MAX_MM      = 600;

// --- Safety / housekeeping --------------------------------------------
constexpr int      HAND_TRACKER_HARD_CAP   = 120;   // ceiling on |PWM| - safety
constexpr uint32_t LOST_TARGET_TIMEOUT_MS  = 500;   // stop if no target for this long
constexpr uint32_t STARTUP_GRACE_MS        = 3000;  // pre-arm countdown
constexpr uint32_t SERIAL_PRINT_MS         = 100;   // diagnostics cadence
constexpr float    INTEGRAL_MAX            = 50.0f; // anti-windup clamp

// ---------------------------------------------------------------------------
//  Internal state
// ---------------------------------------------------------------------------
static float    g_integral       = 0.0f;
static float    g_prevError      = 0.0f;
static uint32_t g_lastTickMs     = 0;        // for dt
static uint32_t g_lastTargetMs   = 0;        // for lost-target watchdog
static uint32_t g_lastPrintMs    = 0;        // for serial throttling
static bool     g_motorsArmed    = false;    // gated by STARTUP_GRACE_MS

// Latest snapshot for diagnostics (filled each loop, printed every SERIAL_PRINT_MS).
struct Telemetry {
    int     target;        // -1 / 0 / +1, or 0 with !acquired
    bool    acquired;
    int     distance;      // mm of the chosen sensor, or -1 if none
    float   headingError;
    int     distanceError; // mm
    float   P, I, D;
    int     output;        // PID output (signed PWM bias)
    int     forwardSpeed;  // translational PWM from distance loop
    int     leftPWM;
    int     rightPWM;
};
static Telemetry g_tele = {};

// ---------------------------------------------------------------------------
//  Target acquisition
//    Pick the NEAREST valid front sensor whose reading is at or below
//    HAND_DETECT_MAX_MM. Returns true if any sensor saw a target.
//      targetPosition: -1 = FL, 0 = FC, +1 = FR
//      distanceMm:     reading of the chosen sensor
// ---------------------------------------------------------------------------
static bool acquireTarget(const TofReadings &r, int &targetPosition, uint16_t &distanceMm) {
    const int      idxs[3]   = { TOF_FL, TOF_FC, TOF_FR };
    const int      poses[3]  = { -1,      0,      +1     };
    int            bestIdx   = -1;
    uint16_t       bestMm    = HAND_DETECT_MAX_MM + 1;

    for (int k = 0; k < 3; ++k) {
        const int i = idxs[k];
        if (!r.online[i]) continue;
        const uint16_t mm = r.mm[i];
        if (mm == TOF_OUT_OF_RANGE) continue;
        if (mm <= HAND_DETECT_MAX_MM && mm < bestMm) {
            bestMm  = mm;
            bestIdx = k;
        }
    }

    if (bestIdx < 0) return false;
    targetPosition = poses[bestIdx];
    distanceMm     = bestMm;
    return true;
}

// ---------------------------------------------------------------------------
//  Differential mix - PHYSICALLY-CORRECT steering convention.
//
//  Sign convention (verified against motors.cpp polarity contract):
//      forward (positive PWM) drives both wheels forward.
//      To pivot the nose toward the RIGHT we spin the LEFT wheel faster
//      than the RIGHT wheel, i.e. leftPWM > rightPWM.
//
//  Therefore for error definition { FL=-1, FC=0, FR=+1 } and Kp > 0:
//      target on RIGHT  -> error = +1  -> output > 0
//                       -> leftPWM = base + output  (faster)
//                       -> rightPWM = base - output (slower)
//                       -> nose pivots RIGHT, toward the hand.  CORRECT.
//      target on LEFT   -> error = -1  -> output < 0
//                       -> left slower, right faster -> nose pivots LEFT.
//
//  Do NOT silently invert this in the production firmware; the Motors::
//  polarity is the single source of truth and must not be re-derived
//  here.
// ---------------------------------------------------------------------------
static inline void mixDifferential(int forwardSpeed, int headingOutput,
                                   int &leftPWM, int &rightPWM) {
    leftPWM  = forwardSpeed + headingOutput;
    rightPWM = forwardSpeed - headingOutput;
    leftPWM  = constrain(leftPWM,  -255, 255);
    rightPWM = constrain(rightPWM, -255, 255);
}

// ---------------------------------------------------------------------------
//  Diagnostics print (throttled to SERIAL_PRINT_MS).
//  Single-line, fixed columns for easy capture into a serial-plotter or
//  CSV pipe during tuning.
// ---------------------------------------------------------------------------
static void printTelemetry() {
    Serial.printf("tgt=%+d acq=%d dist=%4d hErr=%+5.2f dErr=%+5d  "
                  "P=%+7.2f I=%+7.2f D=%+7.2f  out=%+4d  fwd=%+4d  L=%+4d R=%+4d\n",
                  g_tele.target, g_tele.acquired ? 1 : 0, g_tele.distance,
                  g_tele.headingError, g_tele.distanceError,
                  g_tele.P, g_tele.I, g_tele.D,
                  g_tele.output, g_tele.forwardSpeed,
                  g_tele.leftPWM, g_tele.rightPWM);
}

// ---------------------------------------------------------------------------
//  Tuning guide (printed once at startup so the operator has it on-screen).
// ---------------------------------------------------------------------------
static void printTuningGuide() {
    Serial.println();
    Serial.println("---- Hand Tracker PID tuning guide ----");
    Serial.println(" Step 1: Tune PID_KP only (set KI=0, KD=0).");
    Serial.println("         Raise KP until the bot tracks side-to-side hand");
    Serial.println("         motion crisply. Oscillation = KP too high.");
    Serial.println(" Step 2: Add small PID_KD to damp the oscillation.");
    Serial.println("         Start ~ KP/4 and adjust.");
    Serial.println(" Step 3: Add PID_KI ONLY if a persistent steady-state");
    Serial.println("         offset remains (bot consistently lags one side).");
    Serial.println("         Keep KI small; integrator is anti-windup clamped");
    Serial.println("         to +/- INTEGRAL_MAX.");
    Serial.println(" Distance loop: tune DISTANCE_KP / DEADBAND independently");
    Serial.println("         by holding the hand still and watching it hold");
    Serial.println("         a constant gap.");
    Serial.println("---------------------------------------");
    Serial.println();
}

// ===========================================================================
//  setup()
// ===========================================================================
void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println();
    Serial.println("=== TEST: Hand Tracker (PID validation firmware) ===");
    Serial.println("SAFETY: chock the wheels or tether the robot - this");
    Serial.println("        firmware does NOT use the edge sensors.");
    Serial.printf ("Hard motor cap = %d / 255\n", HAND_TRACKER_HARD_CAP);

    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    Wire.setClock(400000);

    Motors::begin();
    Motors::setHardCap(HAND_TRACKER_HARD_CAP);

    const int online = ToF::begin();
    Serial.printf("[ToF] %d / %d sensors online (only FL/FC/FR are used)\n",
                  online, (int)TOF_N);
    if (online == 0) {
        Serial.println("[FATAL] no ToF sensors online - halting.");
        while (true) { Motors::stop(); delay(1000); }
    }

    printTuningGuide();

    // Pre-arm countdown - keeps motors latched off for STARTUP_GRACE_MS so
    // the operator has time to back away / verify chocks.
    Serial.printf("[ARM] starting in %u ms ...\n", STARTUP_GRACE_MS);
    const uint32_t armStart = millis();
    while (millis() - armStart < STARTUP_GRACE_MS) {
        const uint32_t remain = STARTUP_GRACE_MS - (millis() - armStart);
        Serial.printf("  %u ms\n", remain);
        Motors::stop();
        delay(500);
    }
    g_motorsArmed  = true;
    g_lastTickMs   = millis();
    g_lastTargetMs = millis();
    Serial.println("[ARM] motors enabled - PID running.");
}

// ===========================================================================
//  loop()
// ===========================================================================
void loop() {
    // 1) Sample front ToF group (FL/FC/FR only).
    ToF::pollFront();
    const TofReadings r = ToF::readAllSensors();

    // 2) dt for the controller.
    const uint32_t now    = millis();
    const float    dt     = (now - g_lastTickMs) / 1000.0f;
    g_lastTickMs          = now;

    // 3) Target acquisition.
    int      targetPosition = 0;
    uint16_t measuredMm     = 0;
    const bool acquired = acquireTarget(r, targetPosition, measuredMm);

    g_tele.acquired = acquired;
    g_tele.target   = acquired ? targetPosition : 0;
    g_tele.distance = acquired ? (int)measuredMm : -1;

    // 4) Lost-target watchdog. Stop motors, zero PID state, fall through to
    //    diagnostics. We hold here until a target reappears.
    if (!acquired) {
        if (now - g_lastTargetMs > LOST_TARGET_TIMEOUT_MS) {
            Motors::stop();
            g_integral      = 0.0f;
            g_prevError     = 0.0f;
            g_tele.headingError  = 0.0f;
            g_tele.distanceError = 0;
            g_tele.P = g_tele.I = g_tele.D = 0.0f;
            g_tele.output = g_tele.forwardSpeed = 0;
            g_tele.leftPWM = g_tele.rightPWM = 0;
        }
        if (now - g_lastPrintMs >= SERIAL_PRINT_MS) {
            g_lastPrintMs = now;
            printTelemetry();
        }
        return;
    }
    g_lastTargetMs = now;

    // 5) PID heading. Error = discrete target position.
    const float error = (float)targetPosition;

    // Skip the first tick (dt == 0) and any anomalous backward-time tick.
    if (dt > 0.0f) {
        g_integral += error * dt;
        if (g_integral >  INTEGRAL_MAX) g_integral =  INTEGRAL_MAX;
        if (g_integral < -INTEGRAL_MAX) g_integral = -INTEGRAL_MAX;
    }
    const float derivative = (dt > 0.0f) ? (error - g_prevError) / dt : 0.0f;
    g_prevError = error;

    const float P_term = PID_KP * error;
    const float I_term = PID_KI * g_integral;
    const float D_term = PID_KD * derivative;
    const float pidOut = P_term + I_term + D_term;
    const int   headingOutput = constrain((int)pidOut, -255, 255);

    // 6) Distance control: proportional with deadband.
    const int distanceError = (int)measuredMm - (int)TARGET_DISTANCE_MM;
    int forwardSpeed;
    if (abs(distanceError) < (int)DISTANCE_DEADBAND_MM) {
        forwardSpeed = 0; // hold standoff
    } else {
        forwardSpeed = (int)(DISTANCE_KP * (float)distanceError);
        forwardSpeed = constrain(forwardSpeed, -BASE_SPEED, +BASE_SPEED);
    }

    // 7) Differential mix and drive (motors are already hard-capped).
    int leftPWM = 0, rightPWM = 0;
    mixDifferential(forwardSpeed, headingOutput, leftPWM, rightPWM);

    if (g_motorsArmed) {
        Motors::drive(leftPWM, rightPWM);
    } else {
        Motors::stop();
    }

    // 8) Cache telemetry and print on the SERIAL_PRINT_MS cadence.
    g_tele.headingError  = error;
    g_tele.distanceError = distanceError;
    g_tele.P             = P_term;
    g_tele.I             = I_term;
    g_tele.D             = D_term;
    g_tele.output        = headingOutput;
    g_tele.forwardSpeed  = forwardSpeed;
    g_tele.leftPWM       = leftPWM;
    g_tele.rightPWM      = rightPWM;

    if (now - g_lastPrintMs >= SERIAL_PRINT_MS) {
        g_lastPrintMs = now;
        printTelemetry();
    }
}
