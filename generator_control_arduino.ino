/**
 * @file generator_control_arduino.ino
 * @brief Arduino implementation for wind/solar generator control
 * 
 * Production-ready implementation for Arduino platform with hardware
 * interfaces for controlling wind and solar generators.
 * 
 * Hardware Requirements:
 * - Arduino Mega 2560 or similar (for sufficient I/O pins)
 * - Voltage sensors (voltage divider circuits)
 * - Current sensors (ACS712 or similar Hall-effect sensors)
 * - Relays for generator control
 * - PWM-capable pins for brake and MPPT control
 * - Optional: Ethernet shield or WiFi module for networking
 * - Optional: SD card module for data logging
 * 
 * @author MSMaruf
 * @date 2026
 */

#define PLATFORM_ARDUINO
#include "generator_config.h"
#include "generator_control.h"

// Arduino-specific includes
#include <Wire.h>

// Timing variables
unsigned long last_sample_time = 0;
unsigned long last_log_time = 0;
unsigned long last_mppt_time = 0;
unsigned long start_time = 0;

// Telemetry data
telemetry_data_t telemetry;

// Voltage divider calibration (adjust based on your circuit)
const float VOLTAGE_DIVIDER_RATIO = 15.0; // For 0-60V range on 0-5V ADC
const float CURRENT_SENSOR_SENSITIVITY = 0.185; // ACS712-05B: 185mV/A
const float ADC_REFERENCE_VOLTAGE = 5.0;
const int ADC_RESOLUTION = 1024;

// RPM measurement
volatile unsigned long rpm_pulse_count = 0;
unsigned long last_rpm_calc_time = 0;

/**
 * Interrupt service routine for RPM sensor
 */
void rpm_sensor_isr() {
    rpm_pulse_count++;
}

/**
 * Arduino-specific sensor initialization
 */
int sensors_init(void) {
    // Configure ADC pins as inputs
    pinMode(PIN_ADC_WIND_VOLTAGE, INPUT);
    pinMode(PIN_ADC_WIND_CURRENT, INPUT);
    pinMode(PIN_ADC_SOLAR_VOLTAGE, INPUT);
    pinMode(PIN_ADC_SOLAR_CURRENT, INPUT);
    pinMode(PIN_ADC_BATTERY_VOLTAGE, INPUT);
    
    // Configure digital output pins
    pinMode(PIN_RELAY_WIND, OUTPUT);
    pinMode(PIN_RELAY_SOLAR, OUTPUT);
    pinMode(PIN_WIND_BRAKE, OUTPUT);
    pinMode(PIN_SOLAR_PWM, OUTPUT);
    
    // Configure RPM sensor with interrupt
    pinMode(PIN_WIND_RPM_SENSOR, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_WIND_RPM_SENSOR), rpm_sensor_isr, FALLING);
    
    // Set PWM frequency for brake and MPPT control
    // Arduino Mega: Timer 2 for pins 9 and 10
    TCCR2B = TCCR2B & 0b11111000 | 0x01; // Set prescaler to 1 for ~31kHz PWM
    
    // Initialize to safe state
    digitalWrite(PIN_RELAY_WIND, LOW);
    digitalWrite(PIN_RELAY_SOLAR, LOW);
    analogWrite(PIN_WIND_BRAKE, 255); // Full brake
    analogWrite(PIN_SOLAR_PWM, 128);  // 50% duty cycle
    
    Serial.println("Arduino sensors initialized");
    return 0;
}

/**
 * Read voltage from ADC with calibration
 */
float read_adc_voltage(int pin) {
    int adc_value = analogRead(pin);
    float voltage = (adc_value * ADC_REFERENCE_VOLTAGE / ADC_RESOLUTION) * VOLTAGE_DIVIDER_RATIO;
    return voltage;
}

/**
 * Read current from Hall-effect sensor
 */
float read_adc_current(int pin) {
    int adc_value = analogRead(pin);
    float voltage = adc_value * ADC_REFERENCE_VOLTAGE / ADC_RESOLUTION;
    // ACS712: Vout = 2.5V at 0A, sensitivity 185mV/A for 5A version
    float current = (voltage - 2.5) / CURRENT_SENSOR_SENSITIVITY;
    if (current < 0) current = 0; // Ignore negative current
    return current;
}

/**
 * Calculate RPM from pulse count
 */
float calculate_rpm(void) {
    unsigned long current_time = millis();
    unsigned long time_diff = current_time - last_rpm_calc_time;
    
    if (time_diff >= 1000) { // Calculate every second
        // Assuming 1 pulse per revolution
        float rpm = (rpm_pulse_count * 60000.0) / time_diff;
        
        // Reset counters
        rpm_pulse_count = 0;
        last_rpm_calc_time = current_time;
        
        return rpm;
    }
    
    return telemetry.wind.rpm; // Return last known value
}

/**
 * Arduino-specific wind sensor reading
 */
int read_wind_sensors(sensor_data_t* data) {
    if (!data) return -1;
    
    data->voltage = read_adc_voltage(PIN_ADC_WIND_VOLTAGE);
    data->current = read_adc_current(PIN_ADC_WIND_CURRENT);
    data->power = calculate_power(data->voltage, data->current);
    data->rpm = calculate_rpm();
    data->temperature = 25.0; // Placeholder - add temperature sensor if available
    data->timestamp = millis();
    
    return 0;
}

/**
 * Arduino-specific solar sensor reading
 */
int read_solar_sensors(sensor_data_t* data) {
    if (!data) return -1;
    
    data->voltage = read_adc_voltage(PIN_ADC_SOLAR_VOLTAGE);
    data->current = read_adc_current(PIN_ADC_SOLAR_CURRENT);
    data->power = calculate_power(data->voltage, data->current);
    data->rpm = 0;
    data->temperature = 25.0; // Placeholder
    data->timestamp = millis();
    
    return 0;
}

/**
 * Arduino-specific battery sensor reading
 */
int read_battery_sensors(sensor_data_t* data) {
    if (!data) return -1;
    
    data->voltage = read_adc_voltage(PIN_ADC_BATTERY_VOLTAGE);
    data->current = telemetry.wind.current + telemetry.solar.current;
    data->power = calculate_power(data->voltage, data->current);
    data->rpm = 0;
    data->temperature = 25.0;
    data->timestamp = millis();
    
    return 0;
}

/**
 * Arduino-specific wind generator control
 */
int wind_generator_enable(bool enable) {
    digitalWrite(PIN_RELAY_WIND, enable ? HIGH : LOW);
    telemetry.state.wind_enabled = enable;
    
    Serial.print("Wind generator ");
    Serial.println(enable ? "enabled" : "disabled");
    return 0;
}

/**
 * Arduino-specific brake control
 */
int wind_brake_control(uint8_t brake_level) {
    if (brake_level > 100) brake_level = 100;
    
    int pwm_value = (brake_level * 255) / 100;
    analogWrite(PIN_WIND_BRAKE, pwm_value);
    
    telemetry.state.brake_active = (brake_level > 0);
    
    Serial.print("Wind brake set to ");
    Serial.print(brake_level);
    Serial.println("%");
    return 0;
}

/**
 * Arduino-specific solar generator control
 */
int solar_generator_enable(bool enable) {
    digitalWrite(PIN_RELAY_SOLAR, enable ? HIGH : LOW);
    telemetry.state.solar_enabled = enable;
    
    Serial.print("Solar generator ");
    Serial.println(enable ? "enabled" : "disabled");
    return 0;
}

/**
 * Arduino-specific PWM control for MPPT
 */
int solar_set_pwm_duty(float duty) {
    if (duty < 0.0) duty = 0.0;
    if (duty > 1.0) duty = 1.0;
    
    int pwm_value = (int)(duty * 255);
    analogWrite(PIN_SOLAR_PWM, pwm_value);
    
    return 0;
}

/**
 * Setup function - runs once at startup
 */
void setup() {
    // Initialize serial communication
    Serial.begin(115200);
    Serial.println("\n=================================");
    Serial.println("Wind/Solar Generator Controller");
    Serial.println("Arduino Platform");
    Serial.println("=================================\n");
    
    // Initialize system
    start_time = millis();
    last_rpm_calc_time = start_time;
    
    if (generator_init() != 0) {
        Serial.println("ERROR: Initialization failed!");
        while(1); // Halt on error
    }
    
    // Start with safe defaults
    wind_brake_control(100); // Full brake initially
    wind_generator_enable(false);
    solar_generator_enable(false);
    
    delay(1000);
    
    // Enable generators after safe startup
    wind_generator_enable(true);
    solar_generator_enable(true);
    wind_brake_control(0); // Release brake
    
    Serial.println("System ready!");
}

/**
 * Main loop - runs continuously
 */
void loop() {
    unsigned long current_time = millis();
    
    // Update uptime
    telemetry.state.uptime = (current_time - start_time) / 1000;
    
    // Sample sensors at specified rate
    if (current_time - last_sample_time >= SAMPLE_RATE_MS) {
        last_sample_time = current_time;
        
        // Read all sensors
        read_wind_sensors(&telemetry.wind);
        read_solar_sensors(&telemetry.solar);
        read_battery_sensors(&telemetry.battery);
        
        // Apply safety checks and limits
        apply_safety_limits(&telemetry);
    }
    
    // Update MPPT for solar
    if (telemetry.state.solar_enabled && 
        current_time - last_mppt_time >= MPPT_SAMPLE_INTERVAL) {
        last_mppt_time = current_time;
        solar_mppt_update(&telemetry.solar);
    }
    
    // Log telemetry data
    if (current_time - last_log_time >= LOG_INTERVAL_MS) {
        last_log_time = current_time;
        log_telemetry(&telemetry);
    }
}
