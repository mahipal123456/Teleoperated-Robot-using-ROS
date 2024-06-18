# Teleoperated Robot using ROS

This project demonstrates the implementation of a teleoperated robot using ROS (Robot Operating System) and an Arduino microcontroller. The robot can be controlled remotely via keyboard commands, leveraging Wi-Fi for communication and ROS for command processing.

## Table of Contents
- [Introduction](#introduction)
- [Features](#features)
- [Hardware Requirements](#hardware-requirements)
- [Software Requirements](#software-requirements)
- [Setup Instructions](#setup-instructions)
- [Usage](#usage)
- [Code Explanation](#code-explanation)
- [Contributors](#contributors)
- [License](#license)

## Introduction
This project aims to control a robot remotely using ROS. The robot moves in different directions based on keyboard inputs. It uses an Arduino microcontroller for hardware control and ROS for communication and command processing.

## Features
- Remote control of robot using ROS.
- Movement in all directions (forward, backward, left, right, and diagonal).
- Wi-Fi connectivity for ROS communication.

## Hardware Requirements
- Arduino microcontroller (e.g., Arduino Uno)
- Motor driver (e.g., L298N)
- DC motors
- Wi-Fi module (e.g., ESP8266)
- Power supply
- Chassis and wheels
- Connecting wires

## Software Requirements
- ubuntu
- Arduino IDE
- ROS (Robot Operating System)
- rosserial package
- Python

## Setup Instructions
1. **Hardware Setup:**
   - Connect the DC motors to the motor driver.
   - Connect the motor driver to the Arduino.
   - Connect the Wi-Fi module to the Arduino.
   - Power up the setup.

2. **Arduino Code:**
   - Open the Arduino IDE.
   - Create a new file and name it `robot_code.ino`.
   - Copy the provided Arduino code into `robot_code.ino`.
   - Upload the code to the nodemcu .

3. **ROS Setup:**
   - Install ROS on your computer.
   - Install the `rosserial` package .
## Usage
1. Connect to the Wi-Fi network specified in the Arduino code.
2. Open a terminal and start ROS master:
   roscore
3. Launch the rosserial server:
   roslaunch rosserial_server socket.launch
4. Run the teleop_twist_keyboard node:
   rosrun teleop_twist_keyboard teleop_twist_keyboard.py
5. Use the keyboard to control the robot.

##Contributors
MAHIPAL KUMAWAT
Hansraj swami
Vivek Saharan
vinod Choudhary
vishal 