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
