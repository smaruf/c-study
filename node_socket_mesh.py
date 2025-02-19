"""
This module provides functionality to connect an ESP32 device to a Wi-Fi network
and print its network configuration every 10 seconds.

Before running this script, ensure your ESP32 is flashed with MicroPython.
Instructions for flashing and running this script are included in the documentation below.

Flashing MicroPython on ESP32:
1. Download the firmware from https://micropython.org/download/esp32/
2. Install esptool: pip install esptool
3. Connect your ESP32 via USB and check the connection port (e.g., COM3 or /dev/ttyUSB0).
4. Erase the existing firmware: esptool.py --port /dev/ttyUSB0 erase_flash
5. Write the new firmware: esptool.py --chip esp32 --port /dev/ttyUSB0 write_flash -z 0x1000 <path_to_firmware>.bin

Uploading and Running the Script:
Option 1: Using Thonny IDE (recommended for beginners)
- Open Thonny and select MicroPython (ESP32) as the interpreter.
- Copy this script into Thonny and save it as 'main.py' on the ESP32 device.
- Press the 'Run' button to execute.

Option 2: Using ampy
- Save this script as 'main.py' on your computer.
- Upload it to ESP32: ampy --port /dev/ttyUSB0 put main.py
- Use ampy or REPL to run the script: ampy --port /dev/ttyUSB0 run main.py
"""

import network
import time

def connect_to_wifi(ssid, password):
    """
    Connect to the specified Wi-Fi network using the given credentials.
    
    Args:
        ssid (str): The SSID of the Wi-Fi network.
        password (str): The password for the Wi-Fi network.
        
    Returns:
        network.WLAN: A configured and connected WLAN object.
    """
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    if not wlan.isconnected():
        print('Connecting to network...')
        wlan.connect(ssid, password)
        while not wlan.isconnected():
            time.sleep(1)
    print('Network config:', wlan.ifconfig())
    return wlan

def main():
    """
    Main function to manage Wi-Fi connection and periodically
    print IP configuration.
    """
    ssid = 'yourNetworkSSID'
    password = 'yourNetworkPassword'
    wlan = connect_to_wifi(ssid, password)

    while True:
        print('Current IP configuration:', wlan.ifconfig())
        time.sleep(10)

if __name__ == '__main__':
    main()
