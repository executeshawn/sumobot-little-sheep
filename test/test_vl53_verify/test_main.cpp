#include <Arduino.h>
#include <Wire.h>

#define SDA_PIN 8
#define SCL_PIN 9

// Expected assigned addresses
const uint8_t expected[6] = {0x30, 0x31, 0x32, 0x33, 0x34, 0x35};
const char* labels[6] = {"FL", "FC", "FR", "L", "R", "RR"};

void setup() {
    Serial.begin(115200);
    delay(2000);

    Serial.println("\n==============================");
    Serial.println(" VL53L0X 6-SENSOR ADDRESS CHECK");
    Serial.println("==============================");

    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(100000);

    delay(100);

    Serial.println("\n[I2C SCAN]");
    
    bool found[6] = {false,false,false,false,false,false};

    // Scan full I2C range
    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {

            Serial.print("FOUND: 0x");
            Serial.println(addr, HEX);

            // Check if it's one of expected VL53L0X addresses
            for (int i = 0; i < 6; i++) {
                if (addr == expected[i]) {
                    found[i] = true;
                }
            }
        }
    }

    Serial.println("\n[EXPECTED SENSOR MAP CHECK]");

    int okCount = 0;

    for (int i = 0; i < 6; i++) {
        if (found[i]) {
            Serial.print(labels[i]);
            Serial.print(" OK @ 0x");
            Serial.println(expected[i], HEX);
            okCount++;
        } else {
            Serial.print(labels[i]);
            Serial.print(" MISSING @ 0x");
            Serial.println(expected[i], HEX);
        }
    }

    Serial.println("\n==============================");
    Serial.print("TOTAL DETECTED: ");
    Serial.print(okCount);
    Serial.println("/6");

    if (okCount == 6) {
        Serial.println("STATUS: ALL SENSORS CORRECT");
    } else {
        Serial.println("STATUS: MISSING OR MISWIRED SENSOR(S)");
        Serial.println("CHECK XSHUT SEQUENCE OR SWAPPED R/RR");
    }

    Serial.println("==============================\n");
}

void loop() {
    // nothing needed
}