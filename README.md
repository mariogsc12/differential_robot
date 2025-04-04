# ROS2 Differential Robot

This repository contains all the necessary files and resources to build and run a **low-cost homemade differential robot** based on **ROS2**.

## 🗂 Repository Structure

### 📁 Hardware

The 'Hardware/' directory includes:
- CAD models of the full assembly
- .xdf for laser-cut wooden parts
- STL files for 3D-printed components

### 📁 Software

The 'Software/' folder is a complete **ROS2 workspace** ('diffbot_ws') that includes:
- Robot description (URDF, xacro)
- Sensor drivers (IMU, encoders, etc.)
- Motor control nodes
- Teleoperation support
- Launch files for simulation and real robot
- Localization and odometry integration

## 🤖 About the Robot

This is a **differential drive robot** built with affordability and modularity in mind. It serves as a first prototype and learning platform for robotics and ROS2 development.

Components:
- 2x DC Motors with Encoder -> https://amzn.eu/d/aTJvIgY
- Raspberry pi 4 (8GB) -> https://amzn.eu/d/gJoqJDg
- 11.1V LiPo Battery -> https://amzn.eu/d/0fujazv
- RPLIDAR A1M8 -> https://amzn.eu/d/6fyjb1o
- ESP32-S3-DevKitC-1 -> https://amzn.eu/d/iINVQQF
