# ESP32 Attitude & Heading Reference System (AHRS) 🚀

A lightweight, WiFi-enabled AHRS and flight telemetry system built on the ESP32. This project fuses data from an MPU6050 IMU and a BMP180 barometer using a Kalman filter and complementary filters to provide accurate real-time altitude, vertical speed, and spatial orientation.

## Features
* **Sensor Fusion:** Utilizes a custom 1D Kalman filter for altitude/vertical speed and a complementary filter for roll/pitch estimation.
* **Wireless Telemetry:** Built-in HTTP web server serving a responsive dashboard.
* **RESTful JSON API:** Access live telemetry data over your local network.
* **Auto-Calibration:** Calculates and removes gyroscope bias on startup.

## Hardware Requirements
* **Microcontroller:** ESP32 Development Board
* **IMU:** MPU6050 (Accelerometer + Gyroscope)
* **Barometer:** BMP180 or BMP085
* **Connections:** Standard I2C wiring.

## Wiring Diagram
| ESP32 Pin | Sensor Pin | Description |
| :--- | :--- | :--- |
| 3V3 | VCC | 3.3V Power |
| GND | GND | Ground |
| GPIO 21 | SDA | I2C Data |
| GPIO 22 | SCL | I2C Clock |

#Install Dependencies: Open the Arduino IDE and install the following libraries via the Library Manager:

Adafruit MPU6050

Adafruit BMP085

Adafruit Unified Sensor

Configure WiFi: Update the SSID and password in the main sketch:

C++
const char* ssid     = "YOUR_SSID";
const char* password = "YOUR_PASSWORD";
Flash to ESP32: Compile and upload the code to your board.

Calibrate: Leave the board perfectly still on startup for 2-3 seconds while it calibrates the gyroscope.

View Dashboard: Open the Serial Monitor to find the ESP32's local IP address, then navigate to that IP in your web browser.

Author
Nandini Sanavada

License
This project is licensed under the MIT License - see the LICENSE file for details.
