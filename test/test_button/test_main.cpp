// ============================================================================
//  TEST: Start button
// ============================================================================
//
//  Purpose
//    Verify the only remaining UI input: the start button (active-LOW,
//    internal pull-up). Streams the button state continuously so the
//    wiring can be checked at the bench.
//
//    DIP switches and the status LED have been removed from this revision;
//    combat behaviour is fixed in firmware. This test no longer references
//    PIN_SW1/2/3 or PIN_STATUS_LED.
//
//  Safety: this test never enables the motors.
//
//  Flash:  pio test -e esp32s3 -f test_button
//  Watch:  pio device monitor -b 115200
//
#include <Arduino.h>
#include "pins.h"

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n=== TEST: Start button ===");
    Serial.printf("Start button = GPIO %d (press = LOW)\n", PIN_START_BTN);

    pinMode(PIN_START_BTN, INPUT_PULLUP);

    Serial.println("\nPress the start button to see the level change.\n");
    Serial.println("  BTN");
}

void loop() {
    const bool pressed = (digitalRead(PIN_START_BTN) == LOW);
    Serial.printf("  %s\n", pressed ? "PRESSED" : "released");
    delay(150);
}
