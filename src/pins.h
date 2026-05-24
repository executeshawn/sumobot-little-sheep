// ============================================================================
//  pins.h - GPIO assignments for the ESP32-S3 N16R8 sumo robot
// ============================================================================
//
//  ESP32-S3 GPIO map (WROOM-1 / DevKitC-1, N16R8 = octal PSRAM):
//    * Valid GPIOs: 0-21 and 26-48. There is NO GPIO 22-25.
//    * GPIO 26-32 : SPI0/1 flash bus            -> NEVER use.
//    * GPIO 33-37 : Octal PSRAM bus (R8 part)   -> NEVER use with qio_opi.
//    * GPIO 0,3,45,46 : strapping pins          -> avoid / use with care.
//    * GPIO 19,20 : USB D-/D+ (native USB-JTAG) -> avoid if USB needed.
//    * GPIO 43,44 : UART0 TX/RX                  -> avoid if using UART log.
//    * GPIO 39-42 : JTAG (MTCK/MTDO/MTDI/MTMS)   -> usable as GPIO if no JTAG.
//    * GPIO 48    : onboard WS2812 RGB LED on most DevKitC-1 boards.
//
//  NOTE: The classic-ESP32 caution "GPIO 34/36/39 are input-only / go HIGH
//        after reset" does NOT apply to the ESP32-S3. On the S3 these are
//        normal bidirectional GPIOs (subject to the PSRAM/JTAG notes above).
//
//  This revision drops the PCF8574 expander idea and reduces the DIP bank to
//  3 switches, which frees enough pins to avoid every problematic GPIO.
//  Pins deliberately NOT used anywhere: 0 (boot strap), 33-37 (PSRAM),
//  48 (onboard RGB LED).
//
#pragma once

// ---------------------------------------------------------------------------
//  HARDWARE / POWER NOTES (physical build, not firmware)
//    * STBY (GPIO 18): add a 10k pulldown to GND so the driver stays in
//      standby (motors off) during boot/reset before pinMode runs.
//    * VM rail: place a 470uF-1000uF electrolytic bulk capacitor across
//      VM <-> GND right at the TB6612FNG to absorb N20 stall/inrush current
//      and brown-out spikes (add a 0.1uF ceramic in parallel for HF noise).
//    * Grounding: use a STAR-GROUND topology - run separate ground returns
//      from the buck/battery to (a) the motor driver and (b) the ESP32/sensor
//      logic, joining them at a single point at the battery negative. This
//      keeps high motor return currents out of the sensor/I2C ground.
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
//  Motor driver - TB6612FNG
// ---------------------------------------------------------------------------
constexpr int PIN_PWMA = 4;   // Left motor PWM   (LEDC) - safe
constexpr int PIN_AIN1 = 5;   // Left motor dir 1            - safe
constexpr int PIN_AIN2 = 6;   // Left motor dir 2            - safe
constexpr int PIN_PWMB = 7;   // Right motor PWM  (LEDC) - safe
constexpr int PIN_BIN1 = 16;  // Right motor dir 1           - safe
constexpr int PIN_BIN2 = 17;  // Right motor dir 2           - safe
constexpr int PIN_STBY = 18;  // TB6612 standby (HIGH=run)   - safe
                              // HW: add a 10k pulldown to GND on this pin so
                              // the driver stays in standby (motors off)
                              // during boot/reset before pinMode runs.

// ---------------------------------------------------------------------------
//  I2C bus (shared by all 6x VL53L0X)
// ---------------------------------------------------------------------------
constexpr int PIN_I2C_SDA = 8;  // safe
constexpr int PIN_I2C_SCL = 9;  // safe

// ---------------------------------------------------------------------------
//  VL53L0X XSHUT pins (one per sensor, used to bring sensors up one-by-one)
//  Sensor order is fixed and used everywhere as an index 0..5.
// ---------------------------------------------------------------------------
constexpr int PIN_XSHUT_FL = 10;  // Front-left   - safe
constexpr int PIN_XSHUT_FC = 11;  // Front-center - safe
constexpr int PIN_XSHUT_FR = 12;  // Front-right  - safe
constexpr int PIN_XSHUT_L  = 13;  // Left side    - safe
constexpr int PIN_XSHUT_R  = 14;  // Right side   - safe
constexpr int PIN_XSHUT_RR = 15;  // Rear         - safe

// ---------------------------------------------------------------------------
//  QTR-MD-03RC reflectance array (edge detection, RC mode)
//  All three pins are clean GPIOs (38 free, 39/40 are JTAG pins usable as
//  GPIO). In RC mode the line is only ever charged to the MCU's 3.3V, so the
//  5V-powered emitters do not put 5V on these pins.
// ---------------------------------------------------------------------------
constexpr int PIN_QTR_LEFT   = 38;  // safe
constexpr int PIN_QTR_CENTER = 39;  // JTAG MTCK (free as GPIO)
constexpr int PIN_QTR_RIGHT  = 40;  // JTAG MTDO (free as GPIO)
constexpr int QTR_COUNT = 3;

// ---------------------------------------------------------------------------
//  DIP switches - INPUT_PULLUP, ON = LOW. Reduced to 3 (was 6).
//  GPIO 41/42 are JTAG pins (fine as GPIO); 47 is a clean GPIO.
// ---------------------------------------------------------------------------
constexpr int PIN_SW1 = 41;  // aggressive vs defensive   (JTAG MTDI)
constexpr int PIN_SW2 = 42;  // search: spin vs sweep     (JTAG MTMS)
constexpr int PIN_SW3 = 47;  // speed: torque vs fast     - safe

// ---------------------------------------------------------------------------
//  User interface
// ---------------------------------------------------------------------------
constexpr int PIN_START_BTN = 1;  // INPUT_PULLUP, press = LOW. ADC1_CH0, safe.
constexpr int PIN_STATUS_LED = 2; // status LED, safe.
