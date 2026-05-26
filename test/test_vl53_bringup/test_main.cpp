#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>

// ===================== CONFIG =====================
#define SDA_PIN 8
#define SCL_PIN 9
#define SENSOR_COUNT 6

// XSHUT pins. RR moved from GPIO 15 -> GPIO 2 to isolate noisy outputs
// from the OCTAL PSRAM bus on the ESP32-S3 N16R8.
const int xshutPins[SENSOR_COUNT] = {
    10, // FL
    11, // FC
    12, // FR
    13, // L
    14, // R
    2   // RR
};

// Expected I2C addresses
const uint8_t addresses[SENSOR_COUNT] = {
    0x30, 0x31, 0x32, 0x33, 0x34, 0x35
};

const char* labels[SENSOR_COUNT] = {
    "FL", "FC", "FR", "L", "R", "RR"
};

// ===================== SENSOR OBJECTS =====================
VL53L0X sensors[SENSOR_COUNT];

// ===================== HELPERS =====================
void setAllXshutLow() {
    for (int i = 0; i < SENSOR_COUNT; i++) {
        pinMode(xshutPins[i], OUTPUT);
        digitalWrite(xshutPins[i], LOW);
    }
    delay(20);
}

void printI2CScan() {
    Serial.println("\n[I2C SCAN]");
    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            Serial.print("FOUND: 0x");
            Serial.println(addr, HEX);
        }
    }
}

// ===================== SETUP =====================
void setup() {
    Serial.begin(115200);
    delay(2000);

    Serial.println("\n==============================");
    Serial.println(" 6x VL53L0X BRING-UP TEST");
    Serial.println("==============================");

    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(100000);

    // STEP 1: shut everything down
    setAllXshutLow();

    Serial.println("\n[STEP 1] All sensors OFF");

    int onlineCount = 0;

    // STEP 2: sequential bring-up
    for (int i = 0; i < SENSOR_COUNT; i++) {

        Serial.print("\n[STEP 2] Enabling ");
        Serial.println(labels[i]);

        digitalWrite(xshutPins[i], HIGH);
        delay(10);

        if (!sensors[i].init()) {
            Serial.print("FAILED init: ");
            Serial.println(labels[i]);
            continue;
        }

        sensors[i].setAddress(addresses[i]);
        delay(10);

        Serial.print("Assigned ");
        Serial.print(labels[i]);
        Serial.print(" → 0x");
        Serial.println(addresses[i], HEX);

        sensors[i].startContinuous(50);

        onlineCount++;
    }

    // STEP 3: verification scan
    printI2CScan();

    Serial.println("\n[RESULT SUMMARY]");
    Serial.print("Sensors online: ");
    Serial.print(onlineCount);
    Serial.print("/");
    Serial.println(SENSOR_COUNT);

    if (onlineCount == SENSOR_COUNT) {
        Serial.println("STATUS: OK - ALL SENSORS INITIALIZED");
    } else {
        Serial.println("STATUS: ERROR - MISSING OR FAILED SENSOR");
        Serial.println("CHECK XSHUT WIRING OR SENSOR ORDER");
    }

    Serial.println("\n[STREAM MODE START]\n");
}

// ===================== LOOP =====================
void loop() {

    for (int i = 0; i < SENSOR_COUNT; i++) {

        int dist = sensors[i].readRangeContinuousMillimeters();

        Serial.print(labels[i]);
        Serial.print(":");

        if (sensors[i].timeoutOccurred()) {
            Serial.print("TIMEOUT ");
        } else {
            Serial.print(dist);
            Serial.print("mm ");
        }
    }

    Serial.println();
    delay(200);
}