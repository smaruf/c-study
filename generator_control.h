/**
 * @file generator_control.h
 * @brief Core generator control interface for wind and solar systems
 * 
 * Provides platform-independent abstractions for controlling wind and solar
 * generators with safety features, MPPT tracking, and monitoring.
 * 
 * @author MSMaruf
 * @date 2026
 */

#ifndef GENERATOR_CONTROL_H
#define GENERATOR_CONTROL_H

#include <stdint.h>
#include <stdbool.h>

// Data structures
typedef struct {
    float voltage;          // Voltage in volts
    float current;          // Current in amperes
    float power;            // Power in watts
    float rpm;              // RPM (wind generator only)
    float temperature;      // Temperature in Celsius
    uint32_t timestamp;     // Timestamp in milliseconds
} sensor_data_t;

typedef struct {
    bool wind_enabled;      // Wind generator enabled
    bool solar_enabled;     // Solar generator enabled
    bool brake_active;      // Wind brake active
    float mppt_duty_cycle;  // Solar MPPT PWM duty cycle (0-1)
    uint8_t error_code;     // Current error code
    uint32_t uptime;        // System uptime in seconds
} system_state_t;

typedef struct {
    sensor_data_t wind;
    sensor_data_t solar;
    sensor_data_t battery;
    system_state_t state;
} telemetry_data_t;

// Initialization functions
int generator_init(void);
int sensors_init(void);
int network_init(void);

// Sensor reading functions
int read_wind_sensors(sensor_data_t* data);
int read_solar_sensors(sensor_data_t* data);
int read_battery_sensors(sensor_data_t* data);

// Wind generator control
int wind_generator_enable(bool enable);
int wind_brake_control(uint8_t brake_level); // 0-100%
int wind_emergency_stop(void);
bool wind_check_overspeed(float rpm);

// Solar generator control
int solar_generator_enable(bool enable);
int solar_mppt_update(sensor_data_t* solar_data);
int solar_set_pwm_duty(float duty); // 0.0-1.0

// Safety and monitoring
int safety_check_all(telemetry_data_t* data);
int apply_safety_limits(telemetry_data_t* data);
int log_telemetry(telemetry_data_t* data);

// Network functions
int send_telemetry_mesh(telemetry_data_t* data);
int receive_mesh_commands(void);

// Utility functions
float calculate_power(float voltage, float current);
const char* error_code_to_string(uint8_t error_code);

#endif // GENERATOR_CONTROL_H
