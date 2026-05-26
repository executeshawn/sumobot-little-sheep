// ============================================================================
//  TEST: Full system integration (REDUCED SPEED + safety timeout)
// ============================================================================
//
//  Purpose
//    Run the entire firmware stack (multi-rate polling + FSM + real sensors
//    + real motors) but with:
//      * a HARD motor cap that the FSM cannot raise, so the run cannot
//        exceed bench-safe speeds regardless of strategy intent,
//      * a global timeout that latches a STOP fault after FULL_SYSTEM_-
//        TIMEOUT_MS so a forgotten-on robot cannot run the pack flat,
//      * verbose state-transition logging.
//
//    Edge detection retains unconditional priority - this property is the
//    central acceptance criterion of this test (Test_Plan.md ED-02).
//
//  ===========================  S A F E T Y  ============================
//   * Place the robot on the dohyo (or a similar marked surface) with at
//     least 30 cm of clear space on every side before flashing.
//   * Verify the start button is unpressed before powering on.
//   * The test self-stops after the timeout; nevertheless, keep a kill
//     switch (main power toggle) within reach.
//  =======================================================================
//
//  Flash:  pio test -e esp32s3 -f test_full_system
//  Watch:  pio device monitor -b 115200
//
#include <Arduino.h>
#include <Wire.h>
#include "pins.h"
#include "config.h"
#include "motors.h"
#include "sensors_tof.h"
#include "sensors_edge.h"
#include "strategy.h"

constexpr int      FULL_SYSTEM_HARD_CAP    = 130;   // ~ 51 % of full PWM
constexpr uint32_t FULL_SYSTEM_TIMEOUT_MS  = 60000; // 60 s autonomous run

static void i2cScan() {
    Serial.println("[I2C] scan...");
    int found = 0;
    for (uint8_t addr = 1; addr < 127; ++addr) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            Serial.printf("    device @ 0x%02X\n", addr);
            ++found;
        }
    }
    Serial.printf("    total: %d (expect 6 in 0x30..0x35)\n", found);
}

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n=== TEST: Full system integration (REDUCED SPEED) ===");
    Serial.printf("Hard motor cap   = %d / 255\n", FULL_SYSTEM_HARD_CAP);
    Serial.printf("Autonomous limit = %u s\n", FULL_SYSTEM_TIMEOUT_MS / 1000);
    Serial.println("Edge detection has unconditional priority.");

    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    Wire.setClock(400000);

    Motors::begin();
    Motors::setHardCap(FULL_SYSTEM_HARD_CAP); // safety: persistent over FSM

    int online = ToF::begin();
    Serial.printf("[ToF] %d / %d sensors online\n", online, (int)TOF_N);

    Edge::begin();
    Strategy::begin();

    if (online == 0) {
        Strategy::setFault("no ToF sensors online");
    }

    i2cScan();
    Serial.println("[FSM] ready - press start button to begin reduced-speed run");
}

void loop() {
    static uint32_t tEdge = 0, tFront = 0, tSide = 0, tFsm = 0;
    static uint32_t bootMs = millis();
    static State    lastState = State::STOP;
    static bool     timedOut  = false;

    const uint32_t now = millis();

    // ---- SAFETY TIMEOUT --------------------------------------------------
    if (!timedOut && (now - bootMs) >= FULL_SYSTEM_TIMEOUT_MS) {
        timedOut = true;
        Serial.printf("\n[SAFETY] %u s timeout reached - latching STOP\n",
                      FULL_SYSTEM_TIMEOUT_MS / 1000);
        Strategy::setFault("test timeout");
    }

    // ---- FAST: edge has priority -----------------------------------------
    if (now - tEdge >= EDGE_POLL_MS) {
        tEdge = now;
        Edge::poll();
    }

    // ---- MEDIUM: front ToF group ----------------------------------------
    if (now - tFront >= TOF_FRONT_POLL_MS) {
        tFront = now;
        ToF::pollFront();
    }

    // ---- SLOW: side/rear ToF group --------------------------------------
    if (now - tSide >= TOF_SIDE_POLL_MS) {
        tSide = now;
        ToF::pollSide();
    }

    // ---- FSM tick -------------------------------------------------------
    if (now - tFsm >= FSM_UPDATE_MS) {
        tFsm = now;
        Strategy::update();

        State s = Strategy::currentState();
        if (s != lastState) {
            Serial.printf("[FSM] state = %s   (t=%lu ms)\n",
                          Strategy::stateName(s), (unsigned long)now);
            lastState = s;
        }
    }
}
