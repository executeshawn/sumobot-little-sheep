// ============================================================================
//  main.cpp - sumo robot entry point
// ============================================================================
#include <Arduino.h>
#include <Wire.h>
#include "pins.h"
#include "config.h"
#include "motors.h"
#include "sensors_tof.h"
#include "sensors_edge.h"
#include "strategy.h"

static void i2cScan() {
    Serial.println("[I2C] scanning bus...");
    int found = 0;
    for (uint8_t addr = 1; addr < 127; ++addr) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            Serial.printf("[I2C]   device @ 0x%02X\n", addr);
            ++found;
        }
    }
    Serial.printf("[I2C] %d device(s) found "
                  "(expect 0x30..0x35 for the 6 VL53L0X)\n", found);
}

void setup() {
    Serial.begin(115200);
    delay(300);
    Serial.println("\n=== Sumobot Heatseeker boot ===");

    // I2C bus for the ToF ring.
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    Wire.setClock(400000); // 400 kHz fast-mode; VL53L0X supports it

    // Motors first so STBY is defined before anything moves.
    Motors::begin();

    // Bring up the 6 VL53L0X via the XSHUT one-at-a-time sequence.
    int online = ToF::begin();
    Serial.printf("[ToF] %d / %d sensors online\n", online, (int)TOF_N);

    // Edge array.
    Edge::begin();

    // Behaviour FSM + UI pins.
    Strategy::begin();

    // Report final I2C map (post-readdress).
    i2cScan();

    // No ToF sensors => cannot fight safely. Latch a fault -> STOP.
    if (online == 0) {
        Strategy::setFault("no ToF sensors online");
    }

    Serial.println("[FSM] ready - press start button to begin");
}

void loop() {
    static uint32_t tEdge = 0, tFront = 0, tSide = 0, tFsm = 0;
    static State    lastState = State::STOP;

    const uint32_t now = millis();

    // --- FAST: edge detection has top priority, polled first and fastest so
    //     the FSM never drives off the ring while waiting on a ToF read. ---
    if (now - tEdge >= EDGE_POLL_MS) {
        tEdge = now;
        Edge::poll();
    }

    // --- MEDIUM: harvest the front ToF group (engagement-critical). ---
    if (now - tFront >= TOF_FRONT_POLL_MS) {
        tFront = now;
        ToF::pollFront();
    }

    // --- SLOW: harvest the side/rear ToF group (flank awareness). ---
    if (now - tSide >= TOF_SIDE_POLL_MS) {
        tSide = now;
        ToF::pollSide();
    }

    // --- FSM tick: consumes only cached sensor data, so it is non-blocking. ---
    if (now - tFsm >= FSM_UPDATE_MS) {
        tFsm = now;
        Strategy::update();

        State s = Strategy::currentState();
        if (s != lastState) {
            Serial.printf("[FSM] state = %s\n", Strategy::stateName(s));
            lastState = s;
        }
    }
}
