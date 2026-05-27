// ============================================================================
//  TEST: TB6612FNG motor driver - combined motion / drift validation
// ============================================================================
//
//  Purpose
//    Drive BOTH motors together through a deterministic sequence
//    (forward, reverse, left pivot, right pivot, full stop) at a safe
//    reduced PWM so the operator can verify:
//      * direction polarity of each motor (no surprise reversals)
//      * PWM output on both channels (no dead leg)
//      * full-stop coasts cleanly (no creep)
//      * straight-line drift between the two motors
//
//    Pin map comes from lib/Sumobot/pins.h (FINAL VERIFIED):
//      Left  channel: PWMA=4, AIN1=5,  AIN2=6
//      Right channel: PWMB=7, BIN1=47, BIN2=21
//      STBY = 18
//
//    Polarity (VERIFIED at the bench, BOTH channels):
//      forward = IN1 LOW,  IN2 HIGH
//      reverse = IN1 HIGH, IN2 LOW
//
//    Run test_motor_diag FIRST to isolate any single-channel fault before
//    using this combined-motion test.
//
//  ===========================  S A F E T Y  ============================
//   * LIFT WHEELS OFF THE GROUND or block them before powering on.
//   * Test sequences motors indefinitely at PWM 90/255 (both motors).
//   * The hardware 10 k pulldown on STBY keeps the driver off during boot.
//  =======================================================================
//
//  Flash:  pio test -e esp32s3 -f test_motor
//  Watch:  pio device monitor -b 115200
//
#include <Arduino.h>
#include "pins.h"

// LEDC PWM channels - separate from the production Motors module
// (which uses 0/1) so this standalone test is fully independent.
constexpr int      LEDC_CH_LEFT  = 2;  // -> PWMA
constexpr int      LEDC_CH_RIGHT = 3;  // -> PWMB

constexpr uint32_t PWM_FREQ_HZ   = 5000;
constexpr uint8_t  PWM_RES_BIT   = 8;

// TT geared motors are mechanically balanced; no trim compensation is
// applied. Both channels run at the same PWM so any drift is purely
// mechanical and can be observed directly.
constexpr int      MOTOR_PWM     = 90;

constexpr uint32_t SEGMENT_MS    = 2000;
constexpr uint32_t STOP_MS       = 1000;
constexpr uint32_t FULL_STOP_MS  = 3000;

// ---------------------------------------------------------------------------
//  Per-channel primitives. Polarity matches src/motors.cpp:
//    forward = IN1 LOW, IN2 HIGH
//    reverse = IN1 HIGH, IN2 LOW
// ---------------------------------------------------------------------------
static void leftForward(int pwm) {
    digitalWrite(PIN_AIN1, LOW);
    digitalWrite(PIN_AIN2, HIGH);
    ledcWrite(LEDC_CH_LEFT, pwm);
}
static void leftReverse(int pwm) {
    digitalWrite(PIN_AIN1, HIGH);
    digitalWrite(PIN_AIN2, LOW);
    ledcWrite(LEDC_CH_LEFT, pwm);
}
static void rightForward(int pwm) {
    digitalWrite(PIN_BIN1, LOW);
    digitalWrite(PIN_BIN2, HIGH);
    ledcWrite(LEDC_CH_RIGHT, pwm);
}
static void rightReverse(int pwm) {
    digitalWrite(PIN_BIN1, HIGH);
    digitalWrite(PIN_BIN2, LOW);
    ledcWrite(LEDC_CH_RIGHT, pwm);
}

static void stopMotors() {
    ledcWrite(LEDC_CH_LEFT,  0);
    ledcWrite(LEDC_CH_RIGHT, 0);
    digitalWrite(PIN_AIN1, LOW);
    digitalWrite(PIN_AIN2, LOW);
    digitalWrite(PIN_BIN1, LOW);
    digitalWrite(PIN_BIN2, LOW);
}

// ---------------------------------------------------------------------------
//  SETUP - safe power-on order: STBY LOW -> dir pins LOW -> LEDC attach
//          (duty 0) -> STBY HIGH.
// ---------------------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n=== TEST: TB6612FNG combined motion ===");
    Serial.printf("Left  channel : PWMA=GPIO%d  AIN1=GPIO%d  AIN2=GPIO%d  LEDC=%d\n",
                  PIN_PWMA, PIN_AIN1, PIN_AIN2, LEDC_CH_LEFT);
    Serial.printf("Right channel : PWMB=GPIO%d  BIN1=GPIO%d  BIN2=GPIO%d  LEDC=%d\n",
                  PIN_PWMB, PIN_BIN1, PIN_BIN2, LEDC_CH_RIGHT);
    Serial.printf("STBY=GPIO%d (HIGH=run)  PWM=%d/255\n", PIN_STBY, MOTOR_PWM);
    Serial.println("Polarity: forward = IN1 LOW, IN2 HIGH (both channels)");
    Serial.println("\n!!! LIFT WHEELS OFF THE GROUND !!!\n");

    pinMode(PIN_STBY, OUTPUT);
    digitalWrite(PIN_STBY, LOW);

    pinMode(PIN_AIN1, OUTPUT);
    pinMode(PIN_AIN2, OUTPUT);
    pinMode(PIN_BIN1, OUTPUT);
    pinMode(PIN_BIN2, OUTPUT);
    digitalWrite(PIN_AIN1, LOW);
    digitalWrite(PIN_AIN2, LOW);
    digitalWrite(PIN_BIN1, LOW);
    digitalWrite(PIN_BIN2, LOW);

    ledcSetup(LEDC_CH_LEFT,  PWM_FREQ_HZ, PWM_RES_BIT);
    ledcAttachPin(PIN_PWMA,  LEDC_CH_LEFT);
    ledcWrite(LEDC_CH_LEFT,  0);
    ledcSetup(LEDC_CH_RIGHT, PWM_FREQ_HZ, PWM_RES_BIT);
    ledcAttachPin(PIN_PWMB,  LEDC_CH_RIGHT);
    ledcWrite(LEDC_CH_RIGHT, 0);

    digitalWrite(PIN_STBY, HIGH); // bridge enabled, both channels idle
    delay(STOP_MS);
}

// ---------------------------------------------------------------------------
//  LOOP
// ---------------------------------------------------------------------------
void loop() {
    Serial.println("FORWARD");
    leftForward(MOTOR_PWM);
    rightForward(MOTOR_PWM);
    delay(SEGMENT_MS);

    Serial.println("STOP");
    stopMotors();
    delay(STOP_MS);

    Serial.println("REVERSE");
    leftReverse(MOTOR_PWM);
    rightReverse(MOTOR_PWM);
    delay(SEGMENT_MS);

    Serial.println("STOP");
    stopMotors();
    delay(STOP_MS);

    Serial.println("LEFT TURN  (left rev, right fwd)");
    leftReverse(MOTOR_PWM);
    rightForward(MOTOR_PWM);
    delay(SEGMENT_MS);

    Serial.println("STOP");
    stopMotors();
    delay(STOP_MS);

    Serial.println("RIGHT TURN (left fwd, right rev)");
    leftForward(MOTOR_PWM);
    rightReverse(MOTOR_PWM);
    delay(SEGMENT_MS);

    Serial.println("FULL STOP");
    stopMotors();
    delay(FULL_STOP_MS);
}
