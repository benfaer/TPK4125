# Self-Balancing Robot (TPK4125)

<img width="347" height="413" alt="selfbalance_robot" src="https://github.com/user-attachments/assets/c927a674-6bb2-4ef4-ab37-748652c3e360" />

This project presents a self-balancing robot designed and built as part of the **TPK4125** course. The robot uses a PID controller to maintain balance while driving on two wheels, combining sensor processing, motor control, and real-time feedback.

## Features
- **PID-based balance control** using a tuned proportional–integral–derivative regulator  
- **IMU sensor fusion** for tilt angle estimation  
- **DC motor control** with PWM for smooth and responsive actuation  
- **Modular Arduino code structure** for clarity and maintainability  

## How It Works
The IMU continuously measures the robot’s tilt angle. The PID controller calculates the required correction to keep the robot upright, and the motors respond by driving forward or backward to counteract the tilt. The system runs in a real-time control loop to achieve stable balancing.

## File Structure
- `hovedsketch.ino` – Main program and system integration  
- `sensor.ino` – IMU reading and angle estimation  
- `pid.ino` – PID controller implementation  
- `motor.ino` – Motor driver and PWM control  

## Requirements
- Arduino-compatible microcontroller  
- IMU 
- Motor driver  
- DC motors 

## Getting Started
1. Install required Arduino libraries (e.g., MPU6050/IMU libraries).  
2. Upload the project code to the microcontroller.  
3. Power the robot and hold it upright during initialization.  
4. Adjust PID parameters if necessary for optimal stability.

## Contributors
Developed by Benjamin Færestrand and Mohamed Elwalid Fadul
