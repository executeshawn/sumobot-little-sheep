// ============================================================================
//  pins.h - GPIO assignments for the ESP32-S3 N16R8 sumo robot
// ============================================================================
//
//  Board: ESP32-S3 N16R8 (16 MB flash, 8 MB OCTAL PSRAM).
//
//  Hard-blocked GPIOs on this board:
//    * GPIO 26-32 : SPI0/1 flash bus            -> NEVER use.
//    * GPIO 33-37 : Octal PSRAM bus (R8 part)   -> NEVER use with qio_opi.
//    * GPIO 15, 16, 17 : observed to interfere with the OCTAL PSRAM
//        subsystem on this board variant when used for motor PWM /
//        direction pins or other noisy/timing-sensitive peripherals.
//        Treat as RESERVED for this project.
//    * GPIO 0, 3, 45, 46 : strapping pins      -> avoid / use with care.
//    * GPIO 19, 20 : USB D-/D+                 -> avoid if USB needed.
//    * GPIO 43, 44 : UART0 TX/RX                -> avoid if using UART log.
//    * GPIO 48     : onboard WS2812 RGB LED on most DevKitC-1 boards.
//
//  This revision finalises the layout to fully isolate the OCTAL PSRAM
//  bus from the motor subsystem. Status LED and DIP switches are removed;
//  combat behaviour is now deterministic and software-driven.
//
#pragma once

// ---------------------------------------------------------------------------
//  HARDWARE / POWER NOTES
//    * STBY (GPIO 18): add a 10k pulldown to GND so the driver stays in
//      standby (motors off) during boot/reset before pinMode runs.
//    * VM rail: 470uF-1000uF bulk cap + 0.1uF ceramic across VM<->GND at
//      the TB6612FNG to absorb N20 stall/inrush spikes.
//    * Grounding: star-ground topology - separate returns from buck to
//      (a) the motor driver and (b) the ESP32/sensor logic, joined at
//      the battery negative only.
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
//  Motor driver - TB6612FNG  (FINAL VERIFIED layout)
//
//  BIN1/BIN2 migration:
//    16/17 -> 41/42 -> 47/21
//      * 16/17 caused OCTAL PSRAM bus interference.
//      * 41/42 were unreliable as direction outputs on this S3 N16R8
//        build (intermittent latching).
//      * 47/21 are stable; this is the locked map.
//
//  Polarity (verified at the bench, applies to BOTH channels):
//      forward = IN1 LOW,  IN2 HIGH
//      reverse = IN1 HIGH, IN2 LOW
//      brake   = IN1 HIGH, IN2 HIGH
//      coast   = STBY LOW   (or IN1=IN2=LOW with STBY HIGH)
//  See src/motors.cpp applyMotor() for the authoritative implementation.
// ---------------------------------------------------------------------------
constexpr int PIN_PWMA = 4;   // Left  motor PWM (LEDC)
constexpr int PIN_AIN1 = 5;   // Left  motor dir 1
constexpr int PIN_AIN2 = 6;   // Left  motor dir 2
constexpr int PIN_PWMB = 7;   // Right motor PWM (LEDC)
constexpr int PIN_BIN1 = 47;  // Right motor dir 1
constexpr int PIN_BIN2 = 21;  // Right motor dir 2
constexpr int PIN_STBY = 18;  // TB6612 standby (HIGH=run). 10k pulldown to GND.

// ---------------------------------------------------------------------------
//  I2C bus (shared by all 6x VL53L0X)
// ---------------------------------------------------------------------------
constexpr int PIN_I2C_SDA = 8;
constexpr int PIN_I2C_SCL = 9;

// ---------------------------------------------------------------------------
//  VL53L0X XSHUT pins (one per sensor, used for one-by-one bring-up).
//  Sensor order is fixed and used everywhere as an index 0..5.
//  RR moved from GPIO 15 to GPIO 2 to keep all noisy / timing-sensitive
//  digital outputs away from the OCTAL PSRAM bus.
// ---------------------------------------------------------------------------
constexpr int PIN_XSHUT_FL = 10;  // Front-left
constexpr int PIN_XSHUT_FC = 11;  // Front-center
constexpr int PIN_XSHUT_FR = 12;  // Front-right
constexpr int PIN_XSHUT_L  = 13;  // Left side
constexpr int PIN_XSHUT_R  = 14;  // Right side
constexpr int PIN_XSHUT_RR = 2;   // Rear (was GPIO 15 - PSRAM neighbour)

// ---------------------------------------------------------------------------
//  QTR-MD-03RC reflectance array (edge detection, RC mode).
//  Unchanged: LEFT=38, CENTER=39, RIGHT=40. GPIO 39/40 are JTAG pins
//  (MTDO/MTCK) but are usable as GPIO when JTAG is not in use.
// ---------------------------------------------------------------------------
constexpr int PIN_QTR_LEFT   = 38;
constexpr int PIN_QTR_CENTER = 39;
constexpr int PIN_QTR_RIGHT  = 40;
constexpr int QTR_COUNT      = 3;

// ---------------------------------------------------------------------------
//  User interface
// ---------------------------------------------------------------------------
constexpr int PIN_START_BTN = 1;  // INPUT_PULLUP, press = LOW. ADC1_CH0.
