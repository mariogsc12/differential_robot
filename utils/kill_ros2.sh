#!/bin/bash

SCRIPT_PID=$$

processes=(
  "ros2cli.daemon.daemonize"
  "ros2-daemon"
  "rviz2"
  "gzserver"
  "gzclient"
  "robot_state_publisher"
  "controller_server"
  "lifecycle_manager"
  "ros2_daemon"
  "ros2-daemon"
  "ros2_launch"
  "ros2_control_node"
  "witmotion_ros"
  "diffbot_controller"
  "diffbot_bringup"
  "noisy_controller"
  "trajectory_drawer"
  "imu_republisher"
  "diffbot_utils"
  "diffbot_localization"
  "static_transform_publisher"
  "joy_teleop"
  "noisy_controller"
  "controller_manager"
  "trajectory_drawer"
)

for process in "${processes[@]}"; do
  echo "Killing: $process"
  pkill -9 -f "$process"
done
