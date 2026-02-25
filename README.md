# HerdLink Collar V1

HerdLink Collar V1 is a smart edge-computing tracking and health monitoring collar designed for livestock. Built around the Seeed Studio XIAO ESP32-C3, the collar integrates motion sensing, temperature and humidity monitoring, and audio analysis to infer animal behaviors such as grazing, ruminating, walking, and resting. It transmits telemetry data wirelessly via UDP.

## Features

*   **Behavioral Inference:** Utilizes a 6-axis IMU (MPU6050) to calculate activity jerk and variance, categorizing behavior into states like Grazing, Ruminating, Walking, Resting, and High/Low Activity Alerts.
*   **Audio Edge Processing:** Captures audio via an I2S microphone and performs on-device Fast Fourier Transform (FFT) to extract Mel-frequency features and audio magnitude.
*   **Health & Environmental Monitoring:** Measures body temperature using a Dallas DS18B20 sensor and environmental temperature/humidity using an AHT20 sensor.
*   **Power Efficiency:** Implements FreeRTOS tasks, hardware interrupts, and sleep modes (ESP32 Light Sleep) to optimize battery life.
*   **Wireless Telemetry:** Broadcasts structured JSON telemetry payloads over WiFi via UDP for local network collection.

## Hardware Components

*   **Microcontroller:** Seeed Studio XIAO ESP32-C3
*   **Motion Sensor:** MPU6050 (Accelerometer & Gyroscope)
*   **Body Temperature Sensor:** Dallas DS18B20 (OneWire)
*   **Environment Sensor:** AHT20 (Temperature & Humidity)
*   **Microphone:** I2S Microphone
*   **Custom PCB:** KiCad designs included for the main logic board and power management board.

## Project Structure

*   `Board Firmware/Sender/`: C++ PlatformIO project containing the ESP32-C3 edge firmware.
*   `Board Firmware/Reciever/`: Python script to receive and decode UDP telemetry data.
*   `PCB_Design/HerdLinkV1/`: KiCad project and Gerber files for the main collar PCB.
*   `PCB_Design/HerdLinkV1 Power/PowerBoard/`: KiCad project for the power management board.

## Setup and Installation

### 1. Firmware (Sender)

The firmware is built using PlatformIO.

1.  Open the `Board Firmware/Sender` directory in VS Code with the PlatformIO extension installed.
2.  Before flashing, update the WiFi credentials in `src/main.cpp`:
    ```cpp
    const char* WIFI_SSID     = "Your_SSID";
    const char* WIFI_PASSWORD = "Your_PASSWORD";
    ```
3.  Connect the XIAO ESP32-C3 and click the PlatformIO "Upload" button to compile and flash the firmware.

### 2. Receiver (Python script)

The receiver is a lightweight Python script that listens for incoming UDP packets.

1.  Navigate to the receiver directory:
    ```bash
    cd "Board Firmware/Reciever"
    ```
2.  Run the script (requires Python 3):
    ```bash
    python CowPilled.py
    ```
    The script will bind to `0.0.0.0:4210` and print incoming JSON telemetry payloads to the console.

## Telemetry Data Format

The device transmits data in JSON format over UDP. Example payload structure:

```json
{
  "id": "MAC_ADDRESS",
  "seq": 123,
  "ts": 1645000,
  "bt": 38.5,
  "et": 24.2,
  "eh": 45.0,
  "ax": 0.05,
  "ay": 0.98,
  "az": 0.12,
  "gx": 1.50,
  "gy": -0.20,
  "gz": 0.10,
  "gy_var": 0.054,
  "jerk": 45.2,
  "mot_int": 0,
  "err": 0,
  "v": 0.00,
  "s": -65,
  "af": [12.4, 45.2, 5.1, 0.0, 0.0, 0.0, 0.0, 0.0]
}
```

*   `id`: Device MAC address
*   `seq`: Sequence number
*   `ts`: Device uptime timestamp (ms)
*   `bt`: Body temperature (Celsius)
*   `et`: Environmental temperature (Celsius)
*   `eh`: Environmental humidity (%)
*   `ax`, `ay`, `az`: Accelerometer data
*   `gx`, `gy`, `gz`: Gyroscope data
*   `gy_var`: Gyroscope Y-axis variance
*   `jerk`: Calculated activity jerk
*   `mot_int`: Motion interrupt flag (1 if triggered, 0 otherwise)
*   `err`: Error code (currently 0)
*   `v`: Voltage (placeholder)
*   `s`: WiFi RSSI signal strength
*   `af`: Array of Audio Features (Mel bins)
