#!/bin/bash

# Define SSH command to run on the Raspberry Pi
remote_cmd="ssh mario@raspberry 'source /opt/ros/humble/setup.bash; \
export ROS_DOMAIN_ID=2; \
cd /home/mario/differential_robot/diffbot_ws; \
source install/setup.bash; \
ros2 launch diffbot_bringup real_robot.launch.py'"

# Define local command to run joystick teleop
local_cmd="source /opt/ros/humble/setup.bash; \
export ROS_DOMAIN_ID=2; \
cd /home/mario/REPOS/differential_robot/differential_robot/diffbot_ws; \
source install/setup.bash; \
ros2 launch diffbot_controller joystick_teleop.launch.py"

# Launch SSH ROS2 launch in a new gnome-terminal
gnome-terminal -- bash -c "$remote_cmd; exec bash"

# Launch local joystick controller in another gnome-terminal
gnome-terminal -- bash -c "$local_cmd; exec bash"

