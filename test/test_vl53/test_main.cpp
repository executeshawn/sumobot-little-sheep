// ============================================================================
//  TEST: VL53L0X ring bring-up + continuous distance streaming
// ============================================================================
//
//  Purpose
//    1. Exercise the XSHUT sequencing in ToF::begin() and confirm every
//       sensor that comes up gets reassigned off the default 0x29 to its
//       project-specific address (0x30..0x35).
//    2. Print a Serial I2C scan so the operator can verify the address map.
//    3. Stream all six distances continuously so the optics and aim of each
//       sensor can be sanity-checked at the bench.
//    4. Annotate timeout / out-of-range states clearly so a missing sensor
//       cannot be mistaken for "0 mm".
//
//  Safety: this test never enables the motors.
//
//  Flash:  pio test -e esp32s3 -f test_vl53 --without-testing
//  Watch:  pio device monitor -b 115200
//
#include <Arduino.h>
#include <Wire.h>
#include "pins.h"
#include "config.h"
#include "sensors_tof.h"

static void i2cScan(const char *label) {
    Serial.printf("\n[I2C scan - %s]\n", label);
    int found = 0;
    for (uint8_t addr = 1; addr < 127; ++addr) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            Serial.printf("    device @ 0x%02X\n", addr);
            ++found;
        }
    }
    Serial.printf("    total: %d device(s)\n", found);
}

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n=== TEST: VL53L0X ring ===");
    Serial.println("Expected post-bring-up I2C map: 0x30 0x31 0x32 0x33 0x34 0x35");

    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    Wire.setClock(400000);

    // XSHUT sequencing + per-sensor address reassignment + continuous mode
    // (this is the same code path the main firmware uses).
    int online = ToF::begin();
    Serial.printf("\n[Bring-up] %d / %d sensors online\n", online, (int)TOF_N);

    i2cScan("after XSHUT bring-up");

    Serial.println("\n[Streaming distances every 200 ms]");
    Serial.println("  Each cell: <label>:<value>");
    Serial.println("  OFF = sensor never came online");
    Serial.println("  OOR = current reading is out of range / timeout");
}

void loop() {
    ToF::pollFront();
    ToF::pollSide();
    TofReadings r = ToF::readAllSensors();

    const char *label[TOF_N] = { "FL", "FC", "FR", "L ", "R ", "RR" };
    for (int i = 0; i < TOF_N; ++i) {
        if (!r.online[i]) {
            Serial.printf(" %s:OFF ", label[i]);
        } else if (r.mm[i] == TOF_OUT_OF_RANGE) {
            Serial.printf(" %s:OOR ", label[i]);
        } else {
            Serial.printf(" %s:%4u", label[i], r.mm[i]);
        }
    }
    Serial.println();
    delay(200);
}
