#!/bin/bash

# ros2 process to kill
processes=(
  "ros2"
  "from ros2cli.daemon.daemonize import main"
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
)


for process in "${processes[@]}"; do
	pkill -9 -f $process
done


