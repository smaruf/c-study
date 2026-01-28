# Wind and Solar Generator Control System

Production-ready control system for wind turbines and solar panels on embedded platforms (Raspberry Pi, Arduino, ESP32).

## Features

🌟 **Multi-Platform Support**
- ESP32 with WiFi mesh networking
- Raspberry Pi with I2C ADC
- Arduino Mega for standalone operation

⚡ **Wind Generator Control**
- Real-time RPM monitoring
- Overspeed protection with electromagnetic brake
- Dynamic power regulation

☀️ **Solar Panel Management**
- Maximum Power Point Tracking (MPPT)
- Perturb and Observe algorithm
- Automatic voltage optimization

🔋 **Battery Protection**
- Overvoltage/undervoltage protection
- Overcurrent limiting
- Multi-chemistry support (lead-acid, lithium)

🛡️ **Safety Features**
- Emergency shutdown system
- Temperature monitoring
- Fail-safe operation mode
- Comprehensive error handling

📡 **Network Capabilities** (ESP32)
- mDNS-based device discovery
- Mesh network telemetry
- Distributed monitoring

## Quick Start

### Raspberry Pi
```bash
git clone https://github.com/smaruf/c-study.git
cd c-study
make rpi
sudo ./generator_control_rpi
```

### Arduino
1. Open `generator_control_arduino.ino` in Arduino IDE
2. Select board: Arduino Mega 2560
3. Upload to device
4. Monitor via Serial (115200 baud)

### ESP32
```bash
# Setup ESP-IDF environment
. ~/esp/esp-idf/export.sh

# Build and flash
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

## Documentation

- **[DEPLOYMENT.md](DEPLOYMENT.md)** - Complete production deployment guide
- **Hardware requirements and wiring diagrams**
- **Safety considerations and certifications**
- **Configuration and calibration procedures**
- **Troubleshooting and maintenance**

## Project Structure

```
c-study/
├── generator_config.h           # Configuration and pin assignments
├── generator_control.h          # Platform-independent API
├── generator_control.c          # Core control logic
├── generator_control_rpi.c      # Raspberry Pi implementation
├── generator_control_arduino.ino # Arduino implementation
├── generator_control_esp32.c    # ESP32 with mesh networking
├── node_socket_mesh.c           # Original ESP32 mesh network
├── node_socket_mesh.py          # MicroPython WiFi example
├── Makefile                     # Build system
├── DEPLOYMENT.md                # Production deployment guide
└── README.md                    # This file
```

## Configuration

Edit `generator_config.h` to adjust for your system:

```c
// Wind Generator
#define WIND_GEN_MAX_VOLTAGE 48.0f
#define WIND_GEN_OVERSPEED_THRESHOLD 2200

// Solar Panel
#define SOLAR_GEN_RATED_POWER 800.0f
#define MPPT_PERTURB_OBSERVE_ENABLED 1

// Battery Protection
#define BATTERY_MAX_VOLTAGE 54.0f
#define OVER_VOLTAGE_THRESHOLD 56.0f
```

## Hardware Requirements

### Common Components
- Voltage sensors (voltage dividers for 0-60V)
- Current sensors (Hall-effect: ACS712 or similar)
- Solid-state relays (60V, 40A minimum)
- MOSFET PWM controllers
- Electromagnetic brake (wind turbine)

### Platform-Specific
- **Raspberry Pi**: ADS1115 I2C ADC module
- **Arduino**: Arduino Mega 2560 (for sufficient I/O)
- **ESP32**: ESP32 DevKit V1 or similar

See [DEPLOYMENT.md](DEPLOYMENT.md) for complete hardware specifications.

## Safety Warnings

⚠️ **This system controls potentially dangerous equipment**

- High voltage electricity (48V+)
- Rotating machinery (wind turbine)
- Fire risk from electrical faults

**Always:**
- Follow local electrical codes
- Use proper safety equipment
- Test thoroughly before deployment
- Implement mechanical fail-safes
- Have professional inspection for production use

## License

See [LICENSE](LICENSE) file.

## Contributing

Issues and pull requests welcome! Please include:
- Platform and hardware details
- Description of the issue/enhancement
- Test results and logs

## Author

MSMaruf - https://github.com/smaruf

## Version

1.0 - Production-ready release (2026-01-28)
