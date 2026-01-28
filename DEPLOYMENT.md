# Production Deployment Guide

## Wind and Solar Generator Control System

This guide provides comprehensive instructions for deploying the wind and solar generator control system on production hardware (Raspberry Pi, Arduino, ESP32).

---

## Table of Contents

1. [System Overview](#system-overview)
2. [Hardware Requirements](#hardware-requirements)
3. [Safety Considerations](#safety-considerations)
4. [Platform-Specific Setup](#platform-specific-setup)
5. [Configuration](#configuration)
6. [Deployment](#deployment)
7. [Monitoring](#monitoring)
8. [Troubleshooting](#troubleshooting)

---

## System Overview

This control system manages:
- **Wind Generator**: RPM monitoring, overspeed protection, electromagnetic brake control
- **Solar Panels**: Maximum Power Point Tracking (MPPT), voltage/current monitoring
- **Battery Bank**: Charge control, voltage protection, current limiting
- **Network Mesh**: Distributed monitoring and control via mDNS

### Key Features

✅ Multi-platform support (ESP32, Raspberry Pi, Arduino)
✅ Real-time safety monitoring and protection
✅ MPPT for solar optimization
✅ Wind turbine overspeed protection
✅ Mesh network telemetry
✅ Production-grade error handling
✅ Configurable safety limits

---

## Hardware Requirements

### Common Components (All Platforms)

1. **Voltage Sensors**
   - Voltage divider circuits for 0-60V range
   - Recommended: 100kΩ / 10kΩ resistor dividers
   - Capacitor filtering: 100nF ceramic + 10µF electrolytic

2. **Current Sensors**
   - Hall-effect sensors: ACS712-20A or ACS712-30A
   - Alternative: Shunt resistor + differential amplifier

3. **Relays**
   - Solid-state relays (SSR) rated for generator voltage/current
   - Minimum: 60V, 40A rating
   - Recommended: Fotek SSR-40DA or equivalent

4. **Wind Turbine Components**
   - Electromagnetic brake (12V/24V coil)
   - RPM sensor: Hall-effect sensor or optical encoder
   - Brake control: MOSFET module for PWM control

5. **Solar Panel Components**
   - MOSFET-based PWM controller
   - Recommended: IRFZ44N or similar (55V, 49A)
   - Gate driver: TC4427 or bootstrap circuit

### Platform-Specific Requirements

#### Raspberry Pi
- Raspberry Pi 3/4 or newer
- ADS1115 16-bit I2C ADC module (for analog readings)
- Level shifters if using 5V sensors
- Power supply: 5V 3A minimum

#### Arduino
- Arduino Mega 2560 (for sufficient I/O pins)
- Built-in 10-bit ADC (sufficient for most applications)
- Optional: Ethernet shield or WiFi module for networking
- Power supply: 7-12V via barrel jack

#### ESP32
- ESP32 DevKit V1 or similar
- Built-in 12-bit ADC
- Built-in WiFi for mesh networking
- Power supply: 5V via USB or 3.3V regulated

---

## Safety Considerations

### ⚠️ CRITICAL SAFETY WARNINGS

1. **Electrical Hazards**
   - System operates with potentially lethal voltages (48V+)
   - Always disconnect power before maintenance
   - Use proper insulation and enclosures
   - Follow local electrical codes

2. **Mechanical Hazards**
   - Wind turbine blades rotate at high speed
   - Ensure brake system is fail-safe
   - Test emergency stop before deployment

3. **Fire Risk**
   - Use appropriately rated components
   - Install overcurrent protection (fuses/breakers)
   - Monitor temperatures continuously
   - Keep flammable materials away

### Safety Features Implemented

- Overvoltage protection: Disconnects at 56V (configurable)
- Undervoltage warning: Alerts below 40V
- Overcurrent protection: Limits at 50A
- Overspeed protection: Brakes at 2200 RPM
- Over-temperature shutdown: Triggers at 80°C
- Watchdog timer: Resets on system hang (platform-dependent)

### Failsafe Operation

The system defaults to a safe state on:
- Power-on/reset: Brake engaged, generators disconnected
- Communication loss: Maintains last safe state
- Sensor fault: Reduces power output
- Error condition: Logs error and takes protective action

---

## Platform-Specific Setup

### Raspberry Pi Setup

#### 1. Operating System Installation
```bash
# Use Raspberry Pi OS Lite (64-bit recommended)
# Enable I2C for ADC communication
sudo raspi-config
# Interface Options -> I2C -> Enable
```

#### 2. Install Dependencies
```bash
sudo apt-get update
sudo apt-get install -y build-essential git i2c-tools
```

#### 3. Verify I2C ADC
```bash
sudo i2cdetect -y 1
# Should show device at 0x48 (ADS1115)
```

#### 4. Build and Install
```bash
cd /home/runner/work/c-study/c-study
make rpi
sudo make install
```

#### 5. Run at Boot (systemd service)
Create file: `/etc/systemd/system/generator-control.service`
```ini
[Unit]
Description=Wind/Solar Generator Control
After=network.target

[Service]
Type=simple
ExecStart=/usr/local/bin/generator_control_rpi
Restart=always
RestartSec=10
User=root

[Install]
WantedBy=multi-user.target
```

Enable service:
```bash
sudo systemctl enable generator-control
sudo systemctl start generator-control
```

---

### Arduino Setup

#### 1. Install Arduino IDE
- Download from: https://www.arduino.cc/en/software
- Install drivers for your Arduino board

#### 2. Open Project
- File -> Open -> `generator_control_arduino.ino`

#### 3. Configure Board
- Tools -> Board -> Arduino Mega 2560
- Tools -> Port -> Select your COM port

#### 4. Upload
- Click "Upload" button
- Monitor via Tools -> Serial Monitor (115200 baud)

#### Alternative: Command Line
```bash
arduino-cli compile --fqbn arduino:avr:mega generator_control_arduino.ino
arduino-cli upload -p /dev/ttyUSB0 --fqbn arduino:avr:mega
```

---

### ESP32 Setup

#### 1. Install ESP-IDF
```bash
mkdir -p ~/esp
cd ~/esp
git clone --recursive https://github.com/espressif/esp-idf.git
cd esp-idf
./install.sh
. ./export.sh
```

#### 2. Create Project Structure
```bash
mkdir -p ~/generator_project
cd ~/generator_project
idf.py create-project generator_control
```

#### 3. Copy Files
```bash
cp generator_control_esp32.c main/main.c
cp generator_config.h main/
cp generator_control.h main/
cp generator_control.c main/
```

#### 4. Configure WiFi
Edit `generator_config.h`:
```c
#define WIFI_SSID "YourNetworkSSID"
#define WIFI_PASS "YourNetworkPassword"
```

#### 5. Build and Flash
```bash
idf.py build
idf.py -p /dev/ttyUSB0 flash
idf.py -p /dev/ttyUSB0 monitor
```

---

## Configuration

### Adjusting Safety Limits

Edit `generator_config.h`:

```c
// Wind Generator Limits
#define WIND_GEN_MAX_VOLTAGE 48.0f        // Adjust for your system
#define WIND_GEN_MAX_CURRENT 30.0f
#define WIND_GEN_OVERSPEED_THRESHOLD 2200 // RPM

// Solar Generator Limits
#define SOLAR_GEN_MAX_VOLTAGE 48.0f
#define SOLAR_GEN_MAX_CURRENT 20.0f

// Battery Protection
#define BATTERY_MAX_VOLTAGE 54.0f         // Lead-acid: 54V (48V system)
#define BATTERY_MIN_VOLTAGE 42.0f         // Lithium: Adjust accordingly
```

### Calibrating Sensors

#### Voltage Sensors
Measure actual voltage with multimeter, then adjust:
```c
const float VOLTAGE_DIVIDER_RATIO = actual_voltage / measured_adc_voltage;
```

#### Current Sensors
Use known load current to calibrate:
```c
const float CURRENT_SENSOR_SENSITIVITY = voltage_change / current_change;
```

### Network Configuration

For mesh networking (ESP32):
```c
#define WIFI_SSID "your_network"
#define WIFI_PASS "your_password"
#define MDNS_HOSTNAME "generator-controller-01"  // Unique per device
```

---

## Deployment

### Pre-Deployment Checklist

- [ ] All electrical connections secured and insulated
- [ ] Sensors calibrated and tested
- [ ] Safety limits configured for your system
- [ ] Brake mechanism tested (wind turbine)
- [ ] Emergency stop verified
- [ ] Enclosure sealed (IP65 or better for outdoor use)
- [ ] System tested with dummy loads before connecting generators
- [ ] Backup power for controller (UPS/battery backup)

### Initial Startup Procedure

1. **Power On**
   - System starts with brake engaged
   - Generators disconnected
   - 2-second safety delay

2. **Monitor Serial Output**
   - Check for initialization messages
   - Verify sensor readings are reasonable
   - Watch for error messages

3. **Gradual Enablement**
   - Wind generator enables first
   - Brake releases gradually
   - Solar generator enables
   - MPPT begins tracking

4. **Verify Operation**
   - Check voltage/current readings
   - Verify power output calculations
   - Test safety shutdown (if safe to do so)

### Production Operation

Monitor the following:
- Battery voltage (should stay within limits)
- Generator output (should match expected values)
- System uptime (check for unexpected resets)
- Error codes (should be ERROR_NONE)

---

## Monitoring

### Serial Console Output

All platforms log telemetry data every 10 seconds (configurable):

```
=== TELEMETRY LOG ===
Uptime: 3600 seconds
Wind: V=48.20 I=12.50 P=602.50 RPM=850
Solar: V=48.40 I=8.30 P=401.72
Battery: V=48.50 I=20.80
MPPT Duty: 67.23%
Status: OK
====================
```

### Network Monitoring (ESP32/RPI)

ESP32 devices announce themselves via mDNS and can discover other controllers on the network.

Use `avahi-browse` on Linux to see devices:
```bash
avahi-browse -a
```

### Data Logging

For long-term monitoring, redirect output to log file:

Raspberry Pi:
```bash
sudo generator_control_rpi >> /var/log/generator.log 2>&1
```

Or use systemd journal:
```bash
sudo journalctl -u generator-control -f
```

---

## Troubleshooting

### Common Issues

#### 1. Incorrect Sensor Readings
**Symptom**: Voltage/current values are way off
**Solution**: 
- Check wiring connections
- Verify voltage divider ratios
- Recalibrate sensors
- Check ADC reference voltage

#### 2. System Resets/Crashes
**Symptom**: Controller reboots unexpectedly
**Solution**:
- Check power supply stability
- Verify adequate current rating
- Check for electrical noise (add filtering)
- Review serial output for errors before crash

#### 3. Overspeed Protection Triggers Incorrectly
**Symptom**: Wind brake engages when RPM seems normal
**Solution**:
- Calibrate RPM sensor
- Check for noise on RPM input (add pull-up/down)
- Adjust WIND_GEN_OVERSPEED_THRESHOLD
- Verify magnet/sensor alignment

#### 4. MPPT Not Optimizing
**Symptom**: Solar power output is lower than expected
**Solution**:
- Verify PWM output is working (oscilloscope)
- Check MOSFET gate drive circuit
- Adjust MPPT_VOLTAGE_STEP
- Increase MPPT_SAMPLE_INTERVAL if tracking is erratic

#### 5. Network Issues (ESP32)
**Symptom**: Cannot connect to WiFi or mDNS not working
**Solution**:
- Verify WiFi credentials
- Check router supports 2.4GHz (ESP32 doesn't support 5GHz)
- Ensure mDNS is enabled on network
- Check firewall settings

### Error Codes

| Code | Meaning | Action |
|------|---------|--------|
| 0 | ERROR_NONE | Normal operation |
| 1 | ERROR_OVER_VOLTAGE | Disconnects generators |
| 2 | ERROR_UNDER_VOLTAGE | Warning only |
| 3 | ERROR_OVER_CURRENT | Reduces output |
| 4 | ERROR_OVER_TEMPERATURE | Reduces output |
| 5 | ERROR_OVERSPEED | Emergency brake |
| 6 | ERROR_COMMUNICATION | Network issue |
| 7 | ERROR_SENSOR_FAULT | Check sensors |

---

## Maintenance

### Regular Checks (Weekly)
- Inspect electrical connections for corrosion
- Check log files for errors
- Verify sensor readings against multimeter
- Test brake operation

### Periodic Maintenance (Monthly)
- Clean solar panels
- Lubricate wind turbine bearings
- Check battery water levels (if applicable)
- Backup configuration and logs
- Update firmware if available

### Annual Service
- Replace worn components
- Recalibrate all sensors
- Test all safety systems
- Professional electrical inspection

---

## Support and Contributing

For issues, improvements, or questions:
- GitHub: https://github.com/smaruf/c-study
- Create an issue with detailed description
- Include platform, hardware, and log output

---

## License

See LICENSE file in repository root.

---

**Version**: 1.0
**Last Updated**: 2026-01-28
