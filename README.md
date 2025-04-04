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
- 2x DC Motors with Encoder -> https://www.amazon.es/dp/B07WT22RNK?ref=ppx_yo2ov_dt_b_fed_asin_title&th=1
- Raspberry pi 4 (8GB) -> https://www.amazon.es/dp/B0CRM6GGVY?ref=ppx_yo2ov_dt_b_fed_asin_title
- 11.1V LiPo Battery -> https://www.amazon.es/dp/B0C4TLQ9YW?ref=ppx_yo2ov_dt_b_fed_asin_title
- RPLIDAR A1M8 -> https://www.amazon.es/dp/B0C9CB2Z29?ref=ppx_yo2ov_dt_b_fed_asin_title
- ESP32-S3-DevKitC-1 -> https://www.amazon.es/DUBEUYEW-ESP32-S3-DevKitC-1-Desarrollo-Bluetooth-Expansi%C3%B3n/dp/B0CRK47HXM/ref=sr_1_4_sspa?crid=2UYAS9RXXM3P&dib=eyJ2IjoiMSJ9.ytqNzezIWFzcwahgmLEggpQZPyuJLdbguEcv4vAoCxOFhu_Lo3sRA5Q_Ps7IekZVCS7fie5xxeLePnOwsR47LGGT2Qr73BB3Ou48u_eFTSH5YEvaO2LNiS0JFmhfuXpaAChYlqG7aFofVHVr-7fdWLIuFkbLhG8qImlHR2vFsxm9qiwT1Q2GU_A0rah_gLoLe3XYLme9CBFSN3dx7ZVkV0Mn736q3T3cjXPYRWUsCJegCdiDG9BnOWEKrDx9wz31OQolAnd0Wt7wgi-CLzfBYyb9Fj780S-kdGq9UQwQ3FM.owiQ9_YMaG5p1zjzA4aK0mek2T3M2GWzG3oFfFyElX8&dib_tag=se&keywords=esp32-s3-devkitc-1&qid=1743804320&sprefix=esp32%2Bs3%2Bdevki%2Caps%2C115&sr=8-4-spons&sp_csd=d2lkZ2V0TmFtZT1zcF9hdGY&th=1
](https://amzn.eu/d/iINVQQF)
