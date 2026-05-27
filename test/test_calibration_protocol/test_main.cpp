// ============================================================================
//  TEST: Movement calibration protocol
// ============================================================================
//
//  Purpose
//    Guided, interactive bench routine that produces final tuning constants
//    for config.h:
//
//        MOVE_TRIM_LEFT
//        MOVE_TRIM_RIGHT
//        MOVE_MS_PER_DEG_LEFT
//        MOVE_MS_PER_DEG_RIGHT
//
//    This is an open-loop platform (no encoders, no IMU, VL53L0X is an
//    outward-facing opponent ring with no fixed reference surface and no
//    walls in a sumo dohyo). The "calibration" is therefore operator-in-
//    the-loop: the test commands deterministic motions, the operator
//    observes the result and types it in, and the test converts the
//    observation into a tuning constant using a documented heuristic.
//    No quantity printed here is "measured" by the firmware.
//
//  Phases
//    0. SANITY - MANDATORY gate before any calibration math runs. Drives
//                LEFT motor alone forward (2 s), RIGHT motor alone forward
//                (2 s), then both motors reverse together (2 s). Operator
//                confirms each segment matches expectation (Y/N). Any 'N'
//                aborts the whole session before a single trim or ms/deg
//                value can be computed against swapped wiring or inverted
//                polarity. This is the only phase that can stop the run
//                without producing an output block.
//
//    1. DRIFT  - drive forward at TEST_PWM for DRIFT_RUN_MS. Operator
//                types L|R|S followed by magnitude 1..5 (e.g. "L3").
//                Trim of the *faster* side is reduced by 0.02 per unit.
//                Iterates until operator reports straight ("S").
//
//    2. PIVOT  - command a 2000 ms pivot per side at TEST_PWM. Operator
//                types observed rotation in degrees. Computes ms-per-deg
//                as PIVOT_MS / observed_deg.
//
//    3. EMIT   - print a ready-to-paste config.h block with the four
//                constants. Uses `constexpr float` to match the existing
//                config.h style (the project does not use #define for
//                tuning constants). Skipped entirely if sanity failed.
//
//  Safety
//    * Motors::setHardCap(TEST_PWM) is asserted at boot so a bad input
//      cannot run the bot faster than the bench-safe ceiling, regardless
//      of the soft cap or any FSM command.
//    * Every motion phase waits on a Serial keypress before moving.
//    * Motors::stop() is called between every phase and on any unknown
//      input. The deferred stop never relies on a delay() that could be
//      skipped.
//    * No production code is modified. This module is opt-in via
//      `pio test -e esp32s3 -f test_calibration_protocol`.
//
//  Hardware notes
//    * TB6612FNG, polarity locked in src/motors.cpp (verified).
//    * QTR is initialised by Edge::begin() (which is linked in via
//      test_build_src=true) but intentionally NOT used as a drift signal:
//      the array points down at the mat and only reports "you hit the
//      white border", not lateral heading, so it cannot inform a
//      mid-run drift estimate.
//
//  Flash:  pio test -e esp32s3 -f test_calibration_protocol
//  Watch:  pio device monitor -b 115200
//
#include <Arduino.h>
#include "motors.h"

// ----- bench-safety knobs ---------------------------------------------------
constexpr int      TEST_PWM        = 90;    // <= 90 per the calibration spec
constexpr uint32_t SANITY_RUN_MS   = 2000;  // single-channel & reverse runs
constexpr uint32_t DRIFT_RUN_MS    = 2000;  // forward run length (drift phase)
constexpr uint32_t PIVOT_MS        = 2000;  // pivot duration (rotation phase)
constexpr uint32_t INTER_PHASE_MS  = 1500;  // settle gap between phases

// ----- drift heuristic ------------------------------------------------------
constexpr float    TRIM_STEP        = 0.02f;  // per magnitude unit
constexpr float    TRIM_FLOOR       = 0.85f;  // never starve a motor below this
constexpr float    TRIM_CEIL        = 1.00f;  // never request more than caller
constexpr int      MAX_DRIFT_ITERS  = 8;      // safety cap on iteration count

// ----- live tuning state ----------------------------------------------------
static float g_trimLeft  = 1.00f;
static float g_trimRight = 1.00f;
static float g_msPerDegLeft  = 0.0f;
static float g_msPerDegRight = 0.0f;
static bool  g_sanityPassed   = false;
static const char *g_abortReason = nullptr;

// ----- helpers --------------------------------------------------------------
static void drainSerial() {
    while (Serial.available()) (void)Serial.read();
}

// Block until the operator presses a key. Returns the first character
// (or '\0' if the stream closed). Echoes a prompt every 5 s so a missed
// keypress is obvious.
static char waitForKey(const char *prompt) {
    Serial.print(prompt);
    Serial.print("  > ");
    uint32_t lastBeep = millis();
    while (!Serial.available()) {
        if (millis() - lastBeep > 5000) {
            Serial.print("\n  (still waiting) > ");
            lastBeep = millis();
        }
        delay(20);
    }
    char c = (char)Serial.read();
    Serial.println(c);
    return c;
}

// Read a whole line (terminated by \n, \r, or 4 s of inactivity after the
// first byte). Returns the number of characters stored in buf (always
// null-terminated, max bufLen-1).
static int readLine(char *buf, int bufLen) {
    int n = 0;
    while (n < bufLen - 1) {
        if (!Serial.available()) { delay(10); continue; }
        char c = (char)Serial.read();
        if (c == '\n' || c == '\r') break;
        buf[n++] = c;
        // collect anything queued within 4 s of the first byte
        uint32_t deadline = millis() + 4000;
        while (n < bufLen - 1 && millis() < deadline) {
            if (Serial.available()) {
                char d = (char)Serial.read();
                if (d == '\n' || d == '\r') { deadline = 0; break; }
                buf[n++] = d;
                deadline = millis() + 200; // brief inter-char window
            } else {
                delay(5);
            }
        }
        break;
    }
    buf[n] = '\0';
    return n;
}

static float clampTrim(float t) {
    if (t < TRIM_FLOOR) return TRIM_FLOOR;
    if (t > TRIM_CEIL)  return TRIM_CEIL;
    return t;
}

// Apply the current trim to a magnitude PWM. Same scaler model as
// src/movement.cpp so the test produces constants that mean exactly the
// same thing in production.
static int trimmedLeft (int pwm) { return (int)((float)pwm * clampTrim(g_trimLeft));  }
static int trimmedRight(int pwm) { return (int)((float)pwm * clampTrim(g_trimRight)); }

// Strict yes/no prompt. Returns true on Y/y, false on N/n. Any other
// input is rejected and the prompt repeats: a typo must never silently
// pass the sanity gate.
static bool confirmYN(const char *prompt) {
    while (true) {
        Serial.print(prompt);
        Serial.print("  [Y/N] > ");
        char line[8];
        readLine(line, sizeof(line));
        if (line[0] == 'Y' || line[0] == 'y') { Serial.println("  -> YES"); return true; }
        if (line[0] == 'N' || line[0] == 'n') { Serial.println("  -> NO");  return false; }
        Serial.println("  ! please answer Y or N.");
    }
}

// ----- SANITY phase (MANDATORY) --------------------------------------------
// Runs three deterministic motions and asks the operator to confirm each
// matches the expected wheel behaviour. Any 'N' answer sets g_abortReason
// and returns false; the caller must then skip the remaining phases.
//
// What this catches BEFORE we trust physics:
//   * Left/right channel swap (PWMA/PWMB or BIN/AIN mis-wired)
//   * Inverted polarity on one or both channels (forward goes backward)
//   * Dead channel (one wheel doesn't spin while the other does)
//
// What it does NOT catch:
//   * Small mechanical drift between matched-direction motors (that's
//     literally what the drift phase is for)
//   * Reversed wheel mounting that happens to match polarity inversion
//     (the operator's eyes are the final arbiter; phrase prompts so a
//     thinking operator will catch it)
static bool runSanityCheck() {
    Serial.println("\n========== PHASE 0: SANITY CHECK (MANDATORY) ==========");
    Serial.println("Three short motions. Lift the wheels OR confine the bot.");
    Serial.println("Any 'N' answer aborts the session before any calibration");
    Serial.println("math runs against potentially mis-wired hardware.\n");
    drainSerial();

    // --- A. LEFT motor alone, forward ---
    Serial.println("-- SANITY A: LEFT motor only, forward --");
    Serial.println("Expectation: the LEFT wheel spins FORWARD, the right");
    Serial.println("wheel stays still.");
    waitForKey("Press any key to run LEFT-only forward");
    Motors::drive(TEST_PWM, 0);
    delay(SANITY_RUN_MS);
    Motors::stop();
    if (!confirmYN("Did ONLY the left wheel spin, and forward?")) {
        g_abortReason = "sanity A failed: left-only forward not as expected";
        return false;
    }
    delay(500);

    // --- B. RIGHT motor alone, forward ---
    Serial.println("\n-- SANITY B: RIGHT motor only, forward --");
    Serial.println("Expectation: the RIGHT wheel spins FORWARD, the left");
    Serial.println("wheel stays still.");
    waitForKey("Press any key to run RIGHT-only forward");
    Motors::drive(0, TEST_PWM);
    delay(SANITY_RUN_MS);
    Motors::stop();
    if (!confirmYN("Did ONLY the right wheel spin, and forward?")) {
        g_abortReason = "sanity B failed: right-only forward not as expected";
        return false;
    }
    delay(500);

    // --- C. Both motors, reverse ---
    Serial.println("\n-- SANITY C: BOTH motors, reverse --");
    Serial.println("Expectation: BOTH wheels spin REVERSE (bot would back up).");
    waitForKey("Press any key to run both-reverse");
    Motors::reverse(TEST_PWM);
    delay(SANITY_RUN_MS);
    Motors::stop();
    if (!confirmYN("Did BOTH wheels spin REVERSE together?")) {
        g_abortReason = "sanity C failed: both-reverse not as expected";
        return false;
    }

    // --- D. Stop confirmation ---
    Serial.println("\n-- SANITY D: STOP holds --");
    Serial.println("Motors::stop() has been called; bot must be motionless.");
    if (!confirmYN("Is the bot fully stopped (no creep, no PWM hiss)?")) {
        g_abortReason = "sanity D failed: stop did not hold";
        return false;
    }

    Serial.println("\n[SANITY] all four checks PASSED. Direction, polarity,");
    Serial.println("channel mapping, and stop primitive are trustworthy.");
    return true;
}

// ----- DRIFT phase ----------------------------------------------------------
static void runDriftPhase() {
    Serial.println("\n========== PHASE 1: DRIFT CALIBRATION ==========");
    Serial.printf("Forward at PWM=%d for %lu ms. Mark a start line; place\n"
                  "the bot pointing along a known straight reference.\n",
                  TEST_PWM, (unsigned long)DRIFT_RUN_MS);
    Serial.println("After each run, type one of:");
    Serial.println("  L<n>  bot drifted LEFT,  magnitude n=1..5");
    Serial.println("  R<n>  bot drifted RIGHT, magnitude n=1..5");
    Serial.println("  S     straight, accept and continue");
    Serial.println("  Q     abort drift phase, keep current trims");

    for (int iter = 1; iter <= MAX_DRIFT_ITERS; ++iter) {
        Serial.printf("\n-- DRIFT run %d  (TRIM_L=%.2f  TRIM_R=%.2f) --\n",
                      iter, g_trimLeft, g_trimRight);
        drainSerial();
        waitForKey("Press any key when ready to drive forward");

        Motors::drive(trimmedLeft(TEST_PWM), trimmedRight(TEST_PWM));
        delay(DRIFT_RUN_MS);
        Motors::stop();

        Serial.println("[stopped] Enter result:");
        char line[16];
        readLine(line, sizeof(line));

        if (line[0] == 'S' || line[0] == 's') {
            Serial.println("Drift phase converged.");
            return;
        }
        if (line[0] == 'Q' || line[0] == 'q') {
            Serial.println("Drift phase aborted by operator.");
            return;
        }

        bool leftDrift  = (line[0] == 'L' || line[0] == 'l');
        bool rightDrift = (line[0] == 'R' || line[0] == 'r');
        if (!leftDrift && !rightDrift) {
            Serial.println("  ! unrecognised input; expected L<n>|R<n>|S|Q");
            --iter; // don't consume an iteration on a typo
            continue;
        }

        int mag = (line[1] >= '1' && line[1] <= '5') ? (line[1] - '0') : 1;
        float step = TRIM_STEP * (float)mag;

        if (leftDrift) {
            // Bot veered left -> right wheel is faster -> slow the right.
            float before = g_trimRight;
            g_trimRight = clampTrim(g_trimRight - step);
            Serial.printf("  drift LEFT mag=%d  -> TRIM_RIGHT %.2f -> %.2f\n",
                          mag, before, g_trimRight);
        } else {
            // Bot veered right -> left wheel is faster -> slow the left.
            float before = g_trimLeft;
            g_trimLeft = clampTrim(g_trimLeft - step);
            Serial.printf("  drift RIGHT mag=%d -> TRIM_LEFT  %.2f -> %.2f\n",
                          mag, before, g_trimLeft);
        }

        if (g_trimLeft <= TRIM_FLOOR && g_trimRight <= TRIM_FLOOR) {
            Serial.println("  ! both trims at floor; mechanical imbalance "
                           "is beyond what scaling can correct. Stop and "
                           "inspect motors/wheels.");
            return;
        }
    }
    Serial.println("Max drift iterations reached without 'S'; "
                   "accepting current trims as best-effort.");
}

// ----- PIVOT phase ----------------------------------------------------------
// Run one pivot, prompt for observed degrees, return ms-per-deg (or 0 on
// abort / unparseable input).
static float runOnePivot(const char *label, void (*pivotFn)(int)) {
    Serial.printf("\n-- PIVOT %s  (PWM=%d  duration=%lu ms) --\n",
                  label, TEST_PWM, (unsigned long)PIVOT_MS);
    Serial.println("Place bot on a flat surface with room to spin. "
                   "Mark the starting heading.");
    drainSerial();
    waitForKey("Press any key to start pivot");

    pivotFn(TEST_PWM);
    delay(PIVOT_MS);
    Motors::stop();

    Serial.println("[stopped] Measure the angle rotated from the start mark.");
    Serial.print("Observed angle in degrees (or 'Q' to skip): ");
    char line[16];
    readLine(line, sizeof(line));

    if (line[0] == 'Q' || line[0] == 'q' || line[0] == '\0') {
        Serial.println("  pivot skipped; ms-per-deg unchanged.");
        return 0.0f;
    }

    int deg = atoi(line);
    if (deg < 10 || deg > 720) {
        Serial.printf("  ! '%s' not in plausible range [10..720]; skipping.\n",
                      line);
        return 0.0f;
    }
    float msPerDeg = (float)PIVOT_MS / (float)deg;
    Serial.printf("  observed %d deg  -> %lu / %d = %.2f ms/deg\n",
                  deg, (unsigned long)PIVOT_MS, deg, msPerDeg);
    return msPerDeg;
}

static void runPivotPhase() {
    Serial.println("\n========== PHASE 2: PIVOT CALIBRATION ==========");

    float left = runOnePivot("LEFT  (Motors::turnLeft)",  Motors::turnLeft);
    if (left > 0.0f) g_msPerDegLeft = left;
    delay(INTER_PHASE_MS);

    float right = runOnePivot("RIGHT (Motors::turnRight)", Motors::turnRight);
    if (right > 0.0f) g_msPerDegRight = right;
}

// ----- EMIT phase -----------------------------------------------------------
static void emitConfigBlock() {
    Serial.println("\n========== PHASE 3: CONFIG OUTPUT ==========");
    // Defence in depth: even if a future caller invokes this directly
    // without going through setup()'s gate, we refuse to print a config
    // block that wasn't certified by the sanity phase.
    if (!g_sanityPassed) {
        Serial.println("REFUSED: sanity check did not pass; no constants will");
        Serial.println("be emitted. Re-run the test and complete sanity first.");
        return;
    }
    Serial.println("Copy the block below into lib/Sumobot/config.h,");
    Serial.println("replacing the existing MOVE_TRIM_* and MOVE_MS_PER_DEG_*");
    Serial.println("lines:\n");

    Serial.println("// ===== AUTO CALIBRATION OUTPUT =====");
    Serial.printf("constexpr float MOVE_TRIM_LEFT        = %.2ff;\n", g_trimLeft);
    Serial.printf("constexpr float MOVE_TRIM_RIGHT       = %.2ff;\n", g_trimRight);
    if (g_msPerDegLeft > 0.0f) {
        Serial.printf("constexpr float MOVE_MS_PER_DEG_LEFT  = %.2ff;\n",
                      g_msPerDegLeft);
    } else {
        Serial.println("// MOVE_MS_PER_DEG_LEFT  : not measured this run");
    }
    if (g_msPerDegRight > 0.0f) {
        Serial.printf("constexpr float MOVE_MS_PER_DEG_RIGHT = %.2ff;\n",
                      g_msPerDegRight);
    } else {
        Serial.println("// MOVE_MS_PER_DEG_RIGHT : not measured this run");
    }
    Serial.println();
}

// ----- entry point ----------------------------------------------------------
void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n=== TEST: Movement calibration protocol ===");
    Serial.println("Open-loop, operator-in-the-loop calibration.");
    Serial.println("No encoders, no IMU. Every constant comes from your");
    Serial.println("observations entered over Serial.\n");

    Serial.println("!!! LIFT WHEELS during drift-phase setup if you have");
    Serial.println("    not yet marked a straight reference run. Both");
    Serial.println("    phases command real motion at TEST_PWM=90.\n");

    Motors::begin();
    Motors::setHardCap(TEST_PWM); // ceiling that the FSM/strategy could not raise
    Motors::stop();

    // Mandatory sanity gate. Until this passes, we do not trust any
    // direction / channel / polarity assumption, so no calibration math
    // is allowed to run.
    waitForKey("Press any key to begin MANDATORY SANITY CHECK");
    g_sanityPassed = runSanityCheck();
    Motors::stop();

    if (!g_sanityPassed) {
        Serial.println("\n!!! CALIBRATION ABORTED !!!");
        Serial.printf("Reason: %s\n", g_abortReason ? g_abortReason : "unknown");
        Serial.println("Fix the underlying wiring/polarity issue (see");
        Serial.println("lib/Sumobot/motors.cpp and lib/Sumobot/pins.h), then re-flash");
        Serial.println("test. No tuning constants will be printed.");
        return;
    }

    delay(INTER_PHASE_MS);
    waitForKey("Press any key to begin DRIFT phase");
    runDriftPhase();
    Motors::stop();
    delay(INTER_PHASE_MS);

    waitForKey("Press any key to begin PIVOT phase");
    runPivotPhase();
    Motors::stop();
    delay(INTER_PHASE_MS);

    emitConfigBlock();

    Serial.println("=== calibration complete ===");
    Serial.println("Reset the board to run again, or flash production firmware.");
}

void loop() {
    // Calibration is one-shot in setup(); idle here.
    Motors::stop();
    delay(1000);
}
