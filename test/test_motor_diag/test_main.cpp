// ============================================================================
//  TEST: Single-motor diagnostic (one channel at a time)
// ============================================================================
//
//  Purpose
//    Isolate every motor-subsystem variable BEFORE running combined motion
//    logic (test_motor) or the full FSM (test_full_system). At each step,
//    only one channel of the TB6612FNG is exercised so the operator can
//    confirm independently:
//
//      Step A1  LEFT  channel forward    (AIN1=L, AIN2=H, PWMA = test PWM)
//      Step A2  LEFT  channel stop       (duty 0, IN pins LOW)
//      Step A3  LEFT  channel reverse    (AIN1=H, AIN2=L, PWMA = test PWM)
//      Step A4  LEFT  channel brake      (AIN1=H, AIN2=H, duty 255)
//      Step A5  LEFT  channel coast      (STBY LOW)
//
//    Polarity reminder (matches src/motors.cpp - VERIFIED on this build):
//      forward = IN1 HIGH, IN2 LOW    (both channels)
//      reverse = IN1 LOW, IN2 HIGH    (both channels)
//
//      Step B1..B5  same sequence on the RIGHT channel (BIN/PWMB).
//
//      Step C   STBY toggle - drop STBY mid-drive and confirm motor coasts
//                              immediately (validates the safety wiring).
//
//    Each step is announced over Serial and held for SEGMENT_MS so the
//    operator can observe wheel direction and rotation rate. The opposite
//    channel is explicitly held inactive (duty 0, IN pins LOW, STBY HIGH)
//    so a cross-talk failure (mis-mapped channel or shorted PWM line)
//    shows up as the wrong wheel spinning.
//
//  What this test isolates / validates
//    * LEDC PWM output is reaching PWMA and PWMB independently
//    * AIN1/AIN2 (and BIN1/BIN2) polarity matches the documented sign
//      convention (positive = forward)
//    * Left vs right channel mapping is not swapped
//    * STBY truly disables the bridge (coast)
//    * Full-stop primitive leaves no creep
//    * Brake primitive holds the shaft
//
//  ===========================  S A F E T Y  ============================
//   * LIFT WHEELS OFF THE GROUND or block them before flashing.
//   * Only one motor is ever active at a time, at reduced PWM.
//   * The hardware 10k pulldown on STBY keeps the driver off during boot.
//  =======================================================================
//
//  Flash:  pio test -e esp32s3 -f test_motor_diag
//  Watch:  pio device monitor -b 115200
//
#include <Arduino.h>
#include "pins.h"

// LEDC channels - use 2/3 so this test does NOT collide with the
// production Motors module (which uses 0/1) if any future build links
// both. Frequency and resolution match config.h on purpose.
constexpr int      LEDC_CH_LEFT   = 2;
constexpr int      LEDC_CH_RIGHT  = 3;
constexpr uint32_t LEDC_FREQ_HZ   = 5000;
constexpr uint8_t  LEDC_RES_BIT   = 8;

constexpr int      TEST_PWM       = 90;   // ~35 % duty - bench-safe
constexpr uint32_t SEGMENT_MS     = 2500; // hold each phase long enough to read
constexpr uint32_t QUIET_MS       = 1200; // gap between phases for clarity

// ---------------------------------------------------------------------------
//  Low-level primitives - explicit per channel, no abstraction
// ---------------------------------------------------------------------------
static void leftIdle() {
    digitalWrite(PIN_AIN1, LOW);
    digitalWrite(PIN_AIN2, LOW);
    ledcWrite(LEDC_CH_LEFT, 0);
}
static void rightIdle() {
    digitalWrite(PIN_BIN1, LOW);
    digitalWrite(PIN_BIN2, LOW);
    ledcWrite(LEDC_CH_RIGHT, 0);
}
static void allIdle() { leftIdle(); rightIdle(); }

static void leftForward(int pwm) {
    digitalWrite(PIN_AIN1, HIGH);
    digitalWrite(PIN_AIN2, LOW);
    ledcWrite(LEDC_CH_LEFT, pwm);
}
static void leftReverse(int pwm) {
    digitalWrite(PIN_AIN1, LOW);
    digitalWrite(PIN_AIN2, HIGH);
    ledcWrite(LEDC_CH_LEFT, pwm);
}
static void leftBrake() {
    digitalWrite(PIN_AIN1, HIGH);
    digitalWrite(PIN_AIN2, HIGH);
    ledcWrite(LEDC_CH_LEFT, 255);
}

static void rightForward(int pwm) {
    digitalWrite(PIN_BIN1, HIGH);
    digitalWrite(PIN_BIN2, LOW);
    ledcWrite(LEDC_CH_RIGHT, pwm);
}
static void rightReverse(int pwm) {
    digitalWrite(PIN_BIN1, LOW);
    digitalWrite(PIN_BIN2, HIGH);
    ledcWrite(LEDC_CH_RIGHT, pwm);
}
static void rightBrake() {
    digitalWrite(PIN_BIN1, HIGH);
    digitalWrite(PIN_BIN2, HIGH);
    ledcWrite(LEDC_CH_RIGHT, 255);
}

static void enableBridge()  { digitalWrite(PIN_STBY, HIGH); }
static void disableBridge() { digitalWrite(PIN_STBY, LOW); }

static void announce(const char *step, const char *what) {
    Serial.printf("\n--- [%s] %s\n", step, what);
}

// ---------------------------------------------------------------------------
//  SETUP
// ---------------------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    delay(800);
    Serial.println("\n=== TEST: Single-motor diagnostic ===");
    Serial.printf("LEFT  channel : PWMA=GPIO%d  AIN1=GPIO%d  AIN2=GPIO%d  LEDC=%d\n",
                  PIN_PWMA, PIN_AIN1, PIN_AIN2, LEDC_CH_LEFT);
    Serial.printf("RIGHT channel : PWMB=GPIO%d  BIN1=GPIO%d  BIN2=GPIO%d  LEDC=%d\n",
                  PIN_PWMB, PIN_BIN1, PIN_BIN2, LEDC_CH_RIGHT);
    Serial.printf("STBY=GPIO%d  test PWM=%d / 255  segment=%lu ms\n",
                  PIN_STBY, TEST_PWM, (unsigned long)SEGMENT_MS);

    // STBY low first - bridge stays in coast through pin setup. The 10k
    // hardware pulldown also keeps it low between MCU reset and this line.
    pinMode(PIN_STBY, OUTPUT);
    disableBridge();

    pinMode(PIN_AIN1, OUTPUT);
    pinMode(PIN_AIN2, OUTPUT);
    pinMode(PIN_BIN1, OUTPUT);
    pinMode(PIN_BIN2, OUTPUT);
    digitalWrite(PIN_AIN1, LOW);
    digitalWrite(PIN_AIN2, LOW);
    digitalWrite(PIN_BIN1, LOW);
    digitalWrite(PIN_BIN2, LOW);

    // LEDC setup BEFORE asserting STBY HIGH so PWM lines are not floating
    // when the bridge enables.
    ledcSetup(LEDC_CH_LEFT,  LEDC_FREQ_HZ, LEDC_RES_BIT);
    ledcAttachPin(PIN_PWMA,  LEDC_CH_LEFT);
    ledcWrite(LEDC_CH_LEFT,  0);
    ledcSetup(LEDC_CH_RIGHT, LEDC_FREQ_HZ, LEDC_RES_BIT);
    ledcAttachPin(PIN_PWMB,  LEDC_CH_RIGHT);
    ledcWrite(LEDC_CH_RIGHT, 0);

    enableBridge();
    Serial.println("[INIT] bridge enabled, both channels idle.");
    Serial.println("\n!!! LIFT WHEELS OFF THE GROUND BEFORE PROCEEDING !!!");
    delay(2000);
}

// ---------------------------------------------------------------------------
//  LOOP - deterministic sweep, repeats forever
// ---------------------------------------------------------------------------
void loop() {
    // ------- LEFT channel only -------------------------------------------
    announce("A1", "LEFT  forward     (right idle)");
    rightIdle();
    leftForward(TEST_PWM);
    delay(SEGMENT_MS);

    announce("A2", "LEFT  stop        (duty 0, IN pins LOW)");
    leftIdle();
    delay(QUIET_MS);

    announce("A3", "LEFT  reverse     (right idle)");
    leftReverse(TEST_PWM);
    delay(SEGMENT_MS);

    announce("A4", "LEFT  brake       (active short brake)");
    leftBrake();
    delay(SEGMENT_MS);

    announce("A5", "LEFT  coast (STBY LOW)  -- both motors free-wheel");
    leftIdle();   // also clear direction pins
    rightIdle();
    disableBridge();
    delay(SEGMENT_MS);
    enableBridge();
    delay(200);   // settle

    // ------- RIGHT channel only ------------------------------------------
    announce("B1", "RIGHT forward     (left idle)");
    leftIdle();
    rightForward(TEST_PWM);
    delay(SEGMENT_MS);

    announce("B2", "RIGHT stop        (duty 0, IN pins LOW)");
    rightIdle();
    delay(QUIET_MS);

    announce("B3", "RIGHT reverse     (left idle)");
    rightReverse(TEST_PWM);
    delay(SEGMENT_MS);

    announce("B4", "RIGHT brake       (active short brake)");
    rightBrake();
    delay(SEGMENT_MS);

    announce("B5", "RIGHT coast (STBY LOW)");
    leftIdle();
    rightIdle();
    disableBridge();
    delay(SEGMENT_MS);
    enableBridge();
    delay(200);

    // ------- STBY safety check -------------------------------------------
    announce("C", "STBY drop mid-drive - LEFT fwd then bridge OFF");
    leftForward(TEST_PWM);
    rightIdle();
    delay(SEGMENT_MS / 2);
    Serial.println("    >>> dropping STBY now, motor MUST coast immediately");
    disableBridge();
    delay(SEGMENT_MS);
    leftIdle();
    enableBridge();
    delay(QUIET_MS);

    Serial.println("\n=== sweep complete - looping ===");
    delay(QUIET_MS);
}
