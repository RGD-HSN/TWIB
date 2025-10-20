# Two Wheel Inverted Pendulum (TWIB)

 📘 Overview
The **TWIB** project is a self-balancing robot designed to maintain its upright position using feedback control.  
It demonstrates the principles of **stability, control systems, and dynamic balancing**.

## ⚙️ Features
- Balances itself using feedback from sensors
- Moves forward and backward to maintain equilibrium
- Controlled using arduino uno
- Implements a **PID** control algorithm for stability

## Components Used
- **Microcontroller:** Arduino Uno
- **Sensors:** Acceloromeeter MPU6050
- **Actuators:** Two DC motors with motor driver
- **Power Supply**
- **Control Algorithm:** PID controller
- **Software Tools:** Arduino IDE

## How It Works
1. The IMU sensor measures the tilt angle of the robot.
2. The controller (PID) calculates the correction needed to keep the robot upright.
3. The Arduino sends control signals to the motors to move in the direction that restores balance.
4. The system continuously updates this feedback loop to maintain stability.

## Author
**Raghad Hasan**  
Robotics Engineer 
email: ragad.h2002@gmail.com
