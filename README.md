# Drone

This repository holds the firmware and design files for a basic, home-built quadcopter flight controller working on Arduino. The project is an educational and hobby-grade initiative focused on showcasing the general principles of drone control, such as sensor integration and PID (Proportional-Integral-Derivative) control. 

##  Features

* **Custom Flight Controller Firmware:** The central part of the project is an Arduino sketch that captures sensor data, processes it, and drives the motors to level the drone.
* **MPU-6050 Integration:** Utilizes the MPU-6050 6-axis IMU (Inertial Measurement Unit) to capture the drone's angular velocity and orientation (roll, pitch, and yaw).
* **Calibration Routines:** Features a solid, step-by-step calibration procedure for accelerometer and gyroscope to provide accurate measurements by compensating sensor bias and noise.
* **PID Control:** Deploys a PID control loop to ensure stable flight and reaction to user commands, illustrating a robotics and automation concept that is important.
* **Modular Design:** Code and hardware are made to be easily extended and modified.

## Hardware Requirements

* **Arduino Uno:** The brain microcontroller of the flight controller.
* **MPU-6050 IMU:** The 6-axis flight stability sensor.
* **4 x Brushless Motors:** To give lift and directional thrust.
* **4 x Electronic Speed Controllers (ESCs):** To control and drive the brushless motors.
* **Quadcopter Frame:** A frame to hold all the components.
* **LiPo Battery:** A high-discharge-rate battery to drive the drone.
* **Radio Transmitter and Receiver:** For remote control of the drone.
* **Breadboard and Jumper Wires:** For connections and prototyping.


### 2. Installing Dependencies

The project needs the following Arduino libraries. You may install them through the Arduino IDE's Library Manager:

* **Adafruit MPU6050 Library:** For simple communication with the sensor.
* **Adafruit Unified Sensor:** A prerequisite for the MPU6050 library.
* **Wire:** An integrated library for I2C communication.

### 3. Calibration

You have to calibrate the MPU-6050 before a steady flight can be attained. The repository contains specific sketches for the purpose. Calibrate the accelerometer and gyroscope to locate and compensate for sensor biases and offsets.


