// ============================================================================
//  motors.cpp - TB6612FNG control, direct ESP32 LEDC implementation
// ============================================================================
//
//  No external motor library is used: the TB6612FNG is a simple part (two
//  direction pins + one PWM pin per channel, plus a shared STBY) and the
//  ESP32 LEDC peripheral drives the PWM lines directly. This avoids the
//  Arduino-ESP32 core 2.x vs 3.x LEDC API churn that breaks most SparkFun-
//  derived TB6612 libraries.
//
//  TB6612FNG truth table (per channel):
//      IN1   IN2   PWM     mode
//      L     L     X       short brake (both outputs LOW)
//      L     H     PWM     CCW (reverse), modulated
//      H     L     PWM     CW  (forward), modulated
//      H     H     X       short brake
//      STBY=L: high-Z (coast)
//
//  Sign convention here: positive speed = forward (IN1=HIGH, IN2=LOW).
//
//  Requires Arduino-ESP32 core 2.x (uses ledcSetup / ledcAttachPin /
//  ledcWrite). platformio.ini pins espressif32@^6.x for that reason.
//
#include "motors.h"
#include "pins.h"
#include "config.h"
#include <Arduino.h>

namespace {

int g_speedCap = SPEED_FULL;

inline int clampSpeed(int v) {
    if (v >  g_speedCap) v =  g_speedCap;
    if (v < -g_speedCap) v = -g_speedCap;
    return v;
}

// Drive one motor channel with a signed speed in -255..255.
// Sign sets direction via the IN1/IN2 pins; magnitude becomes PWM duty.
void applyMotor(uint8_t ledcCh, int in1Pin, int in2Pin, int spd) {
    const bool fwd  = (spd >= 0);
    int duty = spd >= 0 ? spd : -spd;
    if (duty > 255) duty = 255;
    digitalWrite(in1Pin, fwd ? HIGH : LOW);
    digitalWrite(in2Pin, fwd ? LOW  : HIGH);
    ledcWrite(ledcCh, (uint32_t)duty);
}

// Active short brake (both INs HIGH per the truth table). PWM duty is
// irrelevant in this state but we set it high for fastest current decay.
void brakeMotor(uint8_t ledcCh, int in1Pin, int in2Pin) {
    digitalWrite(in1Pin, HIGH);
    digitalWrite(in2Pin, HIGH);
    ledcWrite(ledcCh, 255);
}

} // namespace

namespace Motors {

void begin() {
    // Direction pins.
    pinMode(PIN_AIN1, OUTPUT);
    pinMode(PIN_AIN2, OUTPUT);
    pinMode(PIN_BIN1, OUTPUT);
    pinMode(PIN_BIN2, OUTPUT);
    digitalWrite(PIN_AIN1, LOW);
    digitalWrite(PIN_AIN2, LOW);
    digitalWrite(PIN_BIN1, LOW);
    digitalWrite(PIN_BIN2, LOW);

    // STBY: pull HIGH to bring the driver out of standby.
    pinMode(PIN_STBY, OUTPUT);
    digitalWrite(PIN_STBY, HIGH);

    // LEDC channels for the two PWM lines, sharing freq/resolution so they
    // can share one LEDC timer with no conflict.
    ledcSetup(MOTOR_CH_LEFT,  MOTOR_PWM_FREQ, MOTOR_PWM_RES);
    ledcAttachPin(PIN_PWMA,   MOTOR_CH_LEFT);
    ledcSetup(MOTOR_CH_RIGHT, MOTOR_PWM_FREQ, MOTOR_PWM_RES);
    ledcAttachPin(PIN_PWMB,   MOTOR_CH_RIGHT);

    g_speedCap = SPEED_FULL;
    drive(0, 0);
}

void setSpeedCap(int cap) {
    if (cap < 0)   cap = 0;
    if (cap > 255) cap = 255;
    g_speedCap = cap;
}

void drive(int leftSpeed, int rightSpeed) {
    // STBY may have been pulled LOW by a previous stop(); re-enable.
    digitalWrite(PIN_STBY, HIGH);
    applyMotor(MOTOR_CH_LEFT,  PIN_AIN1, PIN_AIN2, clampSpeed(leftSpeed));
    applyMotor(MOTOR_CH_RIGHT, PIN_BIN1, PIN_BIN2, clampSpeed(rightSpeed));
}

void forward(int speed)  { speed = abs(speed); drive( speed,  speed); }
void reverse(int speed)  { speed = abs(speed); drive(-speed, -speed); }
void turnLeft(int speed) { speed = abs(speed); drive(-speed,  speed); } // pivot
void turnRight(int speed){ speed = abs(speed); drive( speed, -speed); } // pivot

void brake() {
    digitalWrite(PIN_STBY, HIGH); // brake requires the bridge active
    brakeMotor(MOTOR_CH_LEFT,  PIN_AIN1, PIN_AIN2);
    brakeMotor(MOTOR_CH_RIGHT, PIN_BIN1, PIN_BIN2);
}

void stop() {
    // Coast: zero PWM, both IN pins LOW, and STBY LOW so the bridge goes
    // high-Z (no holding current, motors free-wheel).
    ledcWrite(MOTOR_CH_LEFT,  0);
    ledcWrite(MOTOR_CH_RIGHT, 0);
    digitalWrite(PIN_AIN1, LOW);
    digitalWrite(PIN_AIN2, LOW);
    digitalWrite(PIN_BIN1, LOW);
    digitalWrite(PIN_BIN2, LOW);
    digitalWrite(PIN_STBY, LOW);
}

} // namespace Motors
