// ============================================================================
//  TEST: QTR-MD-03RC edge array - raw values, calibration, threshold check
// ============================================================================
//
//  Purpose
//    1. Show raw RC discharge values (uncalibrated) so the operator can see
//       the dynamic range over black-mat vs white-ring before calibration.
//    2. Run a calibration sweep so the operator can validate that
//       EDGE_THRESHOLD (in config.h) sits well within the calibrated range.
//    3. Stream calibrated readings + WHITE/BLACK interpretation continuously
//       so the threshold can be hand-tuned with confidence.
//
//  Safety: this test never enables the motors. The operator manually slides
//          the QTR over mat and ring at the bench.
//
//  Flash:  pio test -e esp32s3 -f test_qtr
//  Watch:  pio device monitor -b 115200
//
#include <Arduino.h>
#include <QTRSensors.h>
#include "pins.h"
#include "config.h"

static QTRSensors qtr;
static uint16_t   raw[QTR_COUNT];
static uint16_t   cal[QTR_COUNT];

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n=== TEST: QTR-MD-03RC edge array ===");
    Serial.printf("Pins: L=GPIO%d  C=GPIO%d  R=GPIO%d\n",
                  PIN_QTR_LEFT, PIN_QTR_CENTER, PIN_QTR_RIGHT);
    Serial.printf("Configured EDGE_THRESHOLD = %u / 1000\n", EDGE_THRESHOLD);

    const uint8_t pins[QTR_COUNT] = {
        (uint8_t)PIN_QTR_LEFT,
        (uint8_t)PIN_QTR_CENTER,
        (uint8_t)PIN_QTR_RIGHT
    };
    qtr.setTypeRC();
    qtr.setSensorPins(pins, QTR_COUNT);

    // ---------------------------------------------------------------------
    //  PHASE 1 - Raw values, no calibration. Move the array over black mat
    //  and white border to see the swing.
    // ---------------------------------------------------------------------
    Serial.println("\n[Phase 1] Raw RC values (uncalibrated, ~us)");
    Serial.println("  Move the sensor over BLACK mat then over WHITE border.");
    Serial.println("  Higher value = darker (longer discharge time).");
    Serial.println("  Send any character in the Serial monitor to continue.\n");

    uint32_t start = millis();
    while (!Serial.available() && millis() - start < 15000) {
        qtr.read(raw);
        Serial.printf("  raw  L:%4u  C:%4u  R:%4u\n", raw[0], raw[1], raw[2]);
        delay(150);
    }
    while (Serial.available()) Serial.read();

    // ---------------------------------------------------------------------
    //  PHASE 2 - Calibration sweep.
    // ---------------------------------------------------------------------
    Serial.println("\n[Phase 2] Calibration sweep (5 s)");
    Serial.println("Move sensor over BLACK and WHITE surface");

    uint32_t calStart = millis();
    while (millis() - calStart < 5000) {
        qtr.read(raw);        // IMPORTANT
        qtr.calibrate();      // now meaningful
        delay(20);
    }
    Serial.println("\n  Calibration complete.\n");

    // ---------------------------------------------------------------------
    //  PHASE 3 - Calibrated stream + threshold interpretation.
    // ---------------------------------------------------------------------
    Serial.println("[Phase 3] Calibrated values + WHITE/BLACK interpretation");
    Serial.println("  Calibrated scale: 0 = most reflective (WHITE)");
    Serial.println("                    1000 = least reflective (BLACK)");
    Serial.printf("  WHITE = value < EDGE_THRESHOLD (%u)\n\n", EDGE_THRESHOLD);
}

void loop() {
    qtr.readCalibrated(cal);

    auto state = [](uint16_t v) {
        return v < EDGE_THRESHOLD ? "WHITE" : "BLACK";
    };

    Serial.printf(" cal  L:%4u (%s)   C:%4u (%s)   R:%4u (%s)\n",
                  cal[0], state(cal[0]),
                  cal[1], state(cal[1]),
                  cal[2], state(cal[2]));
    delay(120);
}
