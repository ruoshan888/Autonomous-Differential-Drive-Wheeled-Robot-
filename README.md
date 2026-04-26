# Autonomous-Differential-Drive-Wheeled-Robot

This project is an autonomous differential drive robot developed using Arduino for mobile robotics applications.
This project was developed for the course ARR6153 Mobile Robots and Drones.

## Features

- Differential drive motion control  
- PID wheel speed control using encoder feedback  
- Obstacle detection and avoidance  
- Manual and autonomous modes  
- Encoder-based distance estimation  
- Bluetooth communication

## Hardware

- Arduino Uno  
- DC Gear Motors with Encoders  
- L298N Motor Driver  
- Ultrasonic Sensor  
- IR Sensors  
- HC-05 Bluetooth Module  
- Li-Po Battery

## Description

The robot uses two independently driven wheels for movement and turning. Encoder feedback is used for PID speed control to improve straight-line motion.

IR and ultrasonic sensors detect obstacles and trigger avoidance actions such as turning or U-turns.

The robot can operate in:
- Manual mode (user control)
- Autonomous mode (self-navigation)
