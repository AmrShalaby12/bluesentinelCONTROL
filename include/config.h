#pragma once

#include <Arduino.h>

#if __has_include("secrets.h")
#include "secrets.h"
#else
static constexpr char WIFI_SSID[] = "";
static constexpr char WIFI_PASSWORD[] = "";
#endif

namespace config {

static constexpr char DEVICE_NAME[] = "BlueSentinel";
static constexpr char ACCESS_POINT_SSID[] = "BlueSentinel-Setup";
static constexpr char ACCESS_POINT_PASSWORD[] = "BlueWater2026";

// I2C devices
static constexpr uint8_t I2C_SDA_PIN = 21;
static constexpr uint8_t I2C_SCL_PIN = 22;
static constexpr uint8_t LCD_I2C_ADDRESS = 0x27;
static constexpr uint8_t LCD_COLUMNS = 16;
static constexpr uint8_t LCD_ROWS = 2;

// Status LED
#ifdef LED_BUILTIN
static constexpr uint8_t STATUS_LED_PIN = LED_BUILTIN;
#else
static constexpr uint8_t STATUS_LED_PIN = 2;
#endif
static constexpr bool STATUS_LED_ACTIVE_HIGH = true;

// Sensors
static constexpr uint8_t DS18B20_PIN = 13;
static constexpr uint8_t PH_SENSOR_PIN = 34;  // ADC1 only, safe with WiFi on ESP32

// Motor signal pins for four reversible ESCs or motor drivers that accept RC PWM
static constexpr uint8_t MOTOR_FRONT_LEFT_PIN = 25;
static constexpr uint8_t MOTOR_FRONT_RIGHT_PIN = 26;
static constexpr uint8_t MOTOR_REAR_LEFT_PIN = 27;
static constexpr uint8_t MOTOR_REAR_RIGHT_PIN = 14;

// If a motor spins in the wrong direction, flip its value here instead of rewiring.
static constexpr bool INVERT_FRONT_LEFT = false;
static constexpr bool INVERT_FRONT_RIGHT = true;
static constexpr bool INVERT_REAR_LEFT = false;
static constexpr bool INVERT_REAR_RIGHT = true;

// RC PWM ranges for reversible ESCs
static constexpr uint16_t ESC_MIN_US = 1100;
static constexpr uint16_t ESC_NEUTRAL_US = 1500;
static constexpr uint16_t ESC_MAX_US = 1900;
static constexpr uint16_t ESC_ARM_DELAY_MS = 3000;

// Network timing
static constexpr uint32_t WIFI_CONNECT_TIMEOUT_MS = 15000;
static constexpr uint32_t COMMAND_TIMEOUT_MS = 600;
static constexpr uint32_t TELEMETRY_INTERVAL_MS = 250;
static constexpr uint32_t SENSOR_INTERVAL_MS = 150;
static constexpr uint32_t LCD_INTERVAL_MS = 700;
static constexpr uint32_t TEMPERATURE_REQUEST_INTERVAL_MS = 1000;
static constexpr uint32_t TEMPERATURE_CONVERSION_MS = 200;

// Motion tuning
static constexpr float COMMAND_DEADBAND = 0.08f;
static constexpr float MAX_TRANSLATION = 1.0f;
static constexpr float MAX_YAW = 0.75f;
static constexpr float INPUT_EXPO = 0.25f;

// pH tuning. Start with these, then calibrate in buffer solutions.
static constexpr float ADC_REFERENCE_VOLTAGE = 3.30f;
static constexpr uint16_t ADC_MAX_READING = 4095;
static constexpr float PH_NEUTRAL_VOLTAGE = 2.50f;  // Voltage that should read pH 7.00
static constexpr float PH_VOLTS_PER_PH = 0.18f;     // Change after board amplification
static constexpr uint8_t PH_SAMPLE_COUNT = 12;

// Joystick layout from browser Gamepad API
static constexpr uint8_t AXIS_STRAFE = 0;   // Left stick X
static constexpr uint8_t AXIS_SURGE = 1;    // Left stick Y
static constexpr uint8_t AXIS_YAW = 2;      // Right stick X on many pads

}  // namespace config
