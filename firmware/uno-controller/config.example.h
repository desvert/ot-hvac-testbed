#pragma once
#include <Arduino.h>

// ======================================================================
//                         USER CONFIGURATION
// ======================================================================

// Temperature Control
const float COOL_SETPOINT_F   = 74.0f;  // Target indoor temperature (°F)
const float COOL_DEADBAND_F   = 2.0f;   // Dead band around setpoint (±°F)

// Fan PWM Duty Cycles (0-255)
const uint8_t FAN_DUTY_OFF    = 0;      // Fan completely off
const uint8_t FAN_DUTY_LOW    = 80;     // ~31% duty cycle
const uint8_t FAN_DUTY_MED    = 160;    // ~63% duty cycle
const uint8_t FAN_DUTY_HIGH   = 255;    // 100% duty cycle

// CO2 Thresholds (ppm)
const uint16_t CO2_LOW_PPM    = 800;    // Below: minimal fresh air needed
const uint16_t CO2_MED_PPM    = 1200;   // Above: high fresh air + min medium fan

// Damper Servo Angles (degrees, 0-180)
const uint8_t DAMPER_CLOSED   = 0;      // No fresh air intake
const uint8_t DAMPER_LOW      = 45;     // Minimal fresh air
const uint8_t DAMPER_MED      = 90;     // Moderate fresh air
const uint8_t DAMPER_HIGH     = 120;    // Maximum fresh air

// Timing Configuration
const unsigned long SAMPLE_INTERVAL_MS  = 5000;    // Sensor read interval
const unsigned long OCCUPANCY_HOLD_MS   = 30000;   // Hold occupancy after motion
const unsigned long CO2_TIMEOUT_MS      = 1000;    // CO2 UART timeout

// System Behavior
const bool LED_FOLLOWS_OCCUPANCY = true;           // LED indicates occupancy status

// ======================================================================
//                         PIN DEFINITIONS
// ======================================================================

// Analog Inputs
const uint8_t THERMISTOR_PIN   = A0;

// Digital Inputs
const uint8_t PIR_PIN          = 2;      // PIR motion sensor (HIGH = motion)

// Digital Outputs
const uint8_t FAN_PWM_PIN      = 5;      // PWM output to fan MOSFET
const uint8_t DAMPER_SERVO_PIN = 9;      // Servo control for fresh air damper
const uint8_t OCC_LED_PIN      = 13;     // Occupancy indicator LED

// CO2 Sensor UART (T6613)
const uint8_t T6613_RX_PIN     = 10;     // Arduino RX <- T6613 TX
const uint8_t T6613_TX_PIN     = 11;     // Arduino TX -> T6613 RX
