# Makefile for Wind/Solar Generator Control System
# Supports multiple platforms: Raspberry Pi, Arduino, ESP32

# Platform selection
PLATFORM ?= rpi

# Compiler and flags
CC = gcc
CFLAGS = -Wall -Wextra -O2 -std=c11
LDFLAGS = -lm

# Source files
CORE_SRC = generator_control.c
RPI_SRC = generator_control_rpi.c
HEADERS = generator_config.h generator_control.h

# Output
RPI_TARGET = generator_control_rpi

# Arduino and ESP32 have their own build systems
# Arduino: Use Arduino IDE or arduino-cli
# ESP32: Use ESP-IDF (idf.py)

.PHONY: all clean help rpi arduino esp32

# Default target
all: help

# Help message
help:
	@echo "Wind/Solar Generator Control System - Build System"
	@echo ""
	@echo "Available targets:"
	@echo "  make rpi        - Build for Raspberry Pi"
	@echo "  make arduino    - Instructions for Arduino build"
	@echo "  make esp32      - Instructions for ESP32 build"
	@echo "  make clean      - Clean build artifacts"
	@echo ""
	@echo "For cross-platform builds:"
	@echo "  PLATFORM=rpi make rpi"
	@echo ""

# Raspberry Pi build
rpi: $(RPI_TARGET)

$(RPI_TARGET): $(RPI_SRC) $(CORE_SRC) $(HEADERS)
	@echo "Building for Raspberry Pi..."
	$(CC) $(CFLAGS) -DPLATFORM_RPI -o $(RPI_TARGET) $(RPI_SRC) $(CORE_SRC) $(LDFLAGS)
	@echo "Build complete: $(RPI_TARGET)"
	@echo "Run with: sudo ./$(RPI_TARGET)"

# Arduino build instructions
arduino:
	@echo "Arduino Build Instructions:"
	@echo "1. Open Arduino IDE"
	@echo "2. File -> Open -> generator_control_arduino.ino"
	@echo "3. Tools -> Board -> Select your Arduino board (Mega 2560 recommended)"
	@echo "4. Tools -> Port -> Select your COM port"
	@echo "5. Sketch -> Verify/Compile"
	@echo "6. Sketch -> Upload"
	@echo ""
	@echo "Or using arduino-cli:"
	@echo "  arduino-cli compile --fqbn arduino:avr:mega generator_control_arduino.ino"
	@echo "  arduino-cli upload -p /dev/ttyUSB0 --fqbn arduino:avr:mega generator_control_arduino.ino"

# ESP32 build instructions
esp32:
	@echo "ESP32 Build Instructions:"
	@echo "1. Install ESP-IDF: https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/"
	@echo "2. Set up ESP-IDF environment:"
	@echo "   . ~/esp/esp-idf/export.sh"
	@echo "3. Configure WiFi credentials in generator_config.h"
	@echo "4. Create ESP-IDF project structure and add files"
	@echo "5. Build: idf.py build"
	@echo "6. Flash: idf.py -p /dev/ttyUSB0 flash"
	@echo "7. Monitor: idf.py -p /dev/ttyUSB0 monitor"

# Clean build artifacts
clean:
	@echo "Cleaning build artifacts..."
	rm -f $(RPI_TARGET)
	rm -f *.o
	rm -f *.out
	@echo "Clean complete"

# Install (Raspberry Pi only)
install: rpi
	@echo "Installing to /usr/local/bin..."
	sudo cp $(RPI_TARGET) /usr/local/bin/
	@echo "Installation complete"
	@echo "Run with: sudo generator_control_rpi"

# Uninstall
uninstall:
	@echo "Uninstalling..."
	sudo rm -f /usr/local/bin/$(RPI_TARGET)
	@echo "Uninstall complete"
