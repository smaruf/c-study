/**
 * @file generator_config.h
 * @brief Configuration header for wind and solar generator control system
 * 
 * This file contains all configuration parameters for production deployment
 * on Raspberry Pi, Arduino, and ESP32 platforms.
 * 
 * @author MSMaruf
 * @date 2026
 */

#ifndef GENERATOR_CONFIG_H
#define GENERATOR_CONFIG_H

// Platform selection - Define ONE of these
// #define PLATFORM_ESP32
// #define PLATFORM_RPI
// #define PLATFORM_ARDUINO

// Network Configuration
#define WIFI_SSID "yourSSID"
#define WIFI_PASS "yourPassword"
#define MDNS_HOSTNAME "generator-controller"
#define MDNS_INSTANCE "Generator Control Node"

// Wind Generator Configuration
#define WIND_GEN_MAX_VOLTAGE 48.0f        // Maximum safe voltage (V)
#define WIND_GEN_MAX_CURRENT 30.0f        // Maximum safe current (A)
#define WIND_GEN_MAX_RPM 2000             // Maximum safe RPM
#define WIND_GEN_MIN_RPM 100              // Minimum operating RPM
#define WIND_GEN_OVERSPEED_THRESHOLD 2200 // RPM threshold for braking
#define WIND_GEN_CUT_IN_SPEED 3.0f        // Wind speed cut-in (m/s)
#define WIND_GEN_RATED_POWER 1000.0f      // Rated power output (W)

// Solar Generator Configuration
#define SOLAR_GEN_MAX_VOLTAGE 48.0f       // Maximum safe voltage (V)
#define SOLAR_GEN_MAX_CURRENT 20.0f       // Maximum safe current (A)
#define SOLAR_GEN_RATED_POWER 800.0f      // Rated power output (W)
#define MPPT_VOLTAGE_STEP 0.1f            // MPPT voltage step (V)
#define MPPT_SAMPLE_INTERVAL 100          // MPPT sampling interval (ms)
#define MPPT_PERTURB_OBSERVE_ENABLED 1    // Enable P&O MPPT algorithm

// Battery Configuration
#define BATTERY_MAX_VOLTAGE 54.0f         // Maximum battery voltage (V)
#define BATTERY_MIN_VOLTAGE 42.0f         // Minimum battery voltage (V)
#define BATTERY_FLOAT_VOLTAGE 54.0f       // Float charge voltage (V)
#define BATTERY_BULK_VOLTAGE 57.6f        // Bulk charge voltage (V)
#define BATTERY_MAX_CHARGE_CURRENT 40.0f  // Maximum charge current (A)

// Safety Thresholds
#define OVER_VOLTAGE_THRESHOLD 56.0f      // Disconnect threshold (V)
#define UNDER_VOLTAGE_THRESHOLD 40.0f     // Low voltage warning (V)
#define OVER_CURRENT_THRESHOLD 50.0f      // Over current protection (A)
#define OVER_TEMPERATURE_THRESHOLD 80.0f  // Maximum temperature (°C)

// Monitoring Configuration
#define SAMPLE_RATE_MS 1000               // Sensor sampling rate (ms)
#define LOG_INTERVAL_MS 10000             // Data logging interval (ms)
#define TELEMETRY_INTERVAL_MS 5000        // Network telemetry interval (ms)

// GPIO Pin Assignments (Platform-specific)
#ifdef PLATFORM_ESP32
  #define PIN_WIND_BRAKE 25               // Wind turbine brake control
  #define PIN_WIND_RPM_SENSOR 26          // RPM sensor input
  #define PIN_SOLAR_PWM 27                // Solar MPPT PWM output
  #define PIN_RELAY_WIND 32               // Wind generator relay
  #define PIN_RELAY_SOLAR 33              // Solar generator relay
  #define PIN_ADC_WIND_VOLTAGE 34         // Wind voltage sensor (ADC1)
  #define PIN_ADC_WIND_CURRENT 35         // Wind current sensor (ADC1)
  #define PIN_ADC_SOLAR_VOLTAGE 36        // Solar voltage sensor (ADC1)
  #define PIN_ADC_SOLAR_CURRENT 39        // Solar current sensor (ADC1)
  #define PIN_ADC_BATTERY_VOLTAGE 32      // Battery voltage sensor
#endif

#ifdef PLATFORM_ARDUINO
  #define PIN_WIND_BRAKE 9                // Wind turbine brake control (PWM)
  #define PIN_WIND_RPM_SENSOR 2           // RPM sensor input (interrupt)
  #define PIN_SOLAR_PWM 10                // Solar MPPT PWM output
  #define PIN_RELAY_WIND 7                // Wind generator relay
  #define PIN_RELAY_SOLAR 8               // Solar generator relay
  #define PIN_ADC_WIND_VOLTAGE A0         // Wind voltage sensor
  #define PIN_ADC_WIND_CURRENT A1         // Wind current sensor
  #define PIN_ADC_SOLAR_VOLTAGE A2        // Solar voltage sensor
  #define PIN_ADC_SOLAR_CURRENT A3        // Solar current sensor
  #define PIN_ADC_BATTERY_VOLTAGE A4      // Battery voltage sensor
#endif

#ifdef PLATFORM_RPI
  #define PIN_WIND_BRAKE 18               // Wind turbine brake (GPIO18/PWM0)
  #define PIN_WIND_RPM_SENSOR 17          // RPM sensor input (GPIO17)
  #define PIN_SOLAR_PWM 13                // Solar MPPT PWM (GPIO13/PWM1)
  #define PIN_RELAY_WIND 22               // Wind generator relay
  #define PIN_RELAY_SOLAR 23              // Solar generator relay
  // RPI uses I2C ADC (ADS1115) for analog readings
  #define I2C_ADC_ADDRESS 0x48            // I2C address for ADC
#endif

// PWM Configuration
#define PWM_FREQUENCY 25000               // PWM frequency (Hz)
#define PWM_RESOLUTION 8                  // PWM resolution (bits)

// Communication Protocol
#define MESH_SERVICE_NAME "_generator"
#define MESH_SERVICE_PROTOCOL "_tcp"
#define MESH_SERVICE_PORT 8080

// Error Codes
#define ERROR_NONE 0
#define ERROR_OVER_VOLTAGE 1
#define ERROR_UNDER_VOLTAGE 2
#define ERROR_OVER_CURRENT 3
#define ERROR_OVER_TEMPERATURE 4
#define ERROR_OVERSPEED 5
#define ERROR_COMMUNICATION 6
#define ERROR_SENSOR_FAULT 7

#endif // GENERATOR_CONFIG_H
