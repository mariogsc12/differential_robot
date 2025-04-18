#!/bin/bash

# Define SSH command to launch on the Raspberry Pi
remote_cmd="ssh mario@raspberry 'source /opt/ros/humble/setup.bash; \
export ROS_DOMAIN_ID=2; \
cd /home/mario/differential_robot/diffbot_ws; \
source install/setup.bash; \
ros2 launch diffbot_bringup real_robot.launch.py use_joystick:=false'"

# Define local command to launch joystick teleop
local_cmd_1="source /opt/ros/humble/setup.bash; \
export ROS_DOMAIN_ID=2; \
cd /home/mario/REPOS/differential_robot/diffbot_ws; \
source install/setup.bash; \
sleep 5; \
ros2 launch diffbot_controller joystick_teleop.launch.py"

# Define local command to launch rviz
local_cmd_2="source /opt/ros/humble/setup.bash; \
export ROS_DOMAIN_ID=2; \
cd /home/mario/REPOS/differential_robot/diffbot_ws; \
source install/setup.bash; \
sleep 5; \
rviz2 -d ./src/diffbot_bringup/rviz/simulated_robot.rviz"

# Launch SSH ROS2 launch in a new gnome-terminal
gnome-terminal -- bash -c "$remote_cmd; exec bash"

# Launch local joystick controller in another gnome-terminal
gnome-terminal -- bash -c "$local_cmd_1; exec bash"

# Launch local rviz controller in another gnome-terminal
gnome-terminal -- bash -c "$local_cmd_2; exec bash"

