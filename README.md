# ESP32 IMU MUX Project

## Author
Rejwan Himel

## Overview
This project uses an ESP32 to read orientation data from multiple BNO055 IMUs using an I2C multiplexer (TCA9548A).

Currently implemented:
- ESP32 (WROOM-32)
- I2C communication
- TCA9548A MUX
- Two BNO055 IMUs
- Real-time heading, roll, and pitch output

## Hardware Setup
- ESP32 connected to MUX via I2C:
  - SDA → GPIO21
  - SCL → GPIO22
- IMUs connected to different MUX channels:
  - BASE → Channel 1
  - ARM → Channel 7

## Functionality
- Selects MUX channel
- Reads BNO055 orientation data
- Outputs Heading, Roll, Pitch via Serial
- Supports multiple IMUs with same I2C address

## Purpose
This setup is intended for:
- Multi-IMU tracking
- Robotic arm joint sensing
- Calibration and drift analysis

## Next Steps
- Add third IMU (end-effector)
- Implement HOME calibration system
- Improve orientation handling (quaternions)
