#!/bin/bash

# ==========================
# Default values
# ==========================
DEFAULT_SSH_TARGET="mario@raspberry"
DEFAULT_SLEEP_TIME=10
DEFAULT_ROS_DOMAIN_ID=2

# ==========================
# Variables
# ==========================
SSH_TARGET="$DEFAULT_SSH_TARGET"
SLEEP_TIME="$DEFAULT_SLEEP_TIME"

# ==========================
# Help
# ==========================
print_help() {
    echo ""
    echo "Usage: $0 [options]"
    echo ""
    echo "Options:"
    echo "  --help               Show this help message"
    echo "  --ssh                SSH target in format username@hostname_or_ip (default: $DEFAULT_SSH_TARGET)"
    echo "  --sleep              Time between launch remote and local processes (default: $DEFAULT_SLEEP_TIME)"
    echo ""
    echo ""
}

# ==========================
# Parse command line
# ==========================
parse_command_line() {
    while [[ $# -gt 0 ]]; do
        case $1 in
            --help)
                print_help
                exit 0
                ;;
            --ssh)
                SSH_TARGET="$2"
                shift 2
                ;;
            --sleep)
                SLEEP_TIME="$2"
                shift 2
                ;;
            *)
                echo "Unknown option: $1"
                print_help
                exit 1
                ;;
        esac
    done
}

# ==========================
# Launch remote processes using ssh
# ==========================
remote_terminal() {
    cmd="ssh ${SSH_TARGET} 'source /opt/ros/humble/setup.bash; \
    export ROS_DOMAIN_ID=${DEFAULT_ROS_DOMAIN_ID}; \
    cd /home/mario/differential_robot/diffbot_ws; \
    source install/setup.bash; \
    ros2 launch diffbot_bringup real_robot.launch.py use_joystick:=false use_slam:=true '"

    gnome-terminal -- bash -c "$cmd; exec bash"
}

remote_terminal_bno055_imu() {
    cmd="ssh ${SSH_TARGET} 'source /opt/ros/humble/setup.bash; \
    cd /home/mario/differential_robot/diffbot_ws; \
    source install/setup.bash; \
    ros2 launch diffbot_bringup bno055.launch.py '"

    gnome-terminal -- bash -c "$cmd; exec bash"
}

remote_terminal_rplidar() {
    cmd="ssh ${SSH_TARGET} 'source /opt/ros/humble/setup.bash; \
    cd /home/mario/differential_robot/diffbot_ws; \
    source install/setup.bash; \
    ros2 launch diffbot_bringup rplidar_a1.launch.py '"

    gnome-terminal -- bash -c "$cmd; exec bash"
}

# ==========================
# Launch local processes
# ==========================
local_terminal_joystick() {
    cmd="source /opt/ros/humble/setup.bash; \
    export ROS_DOMAIN_ID=${DEFAULT_ROS_DOMAIN_ID}; \
    cd /home/mario/REPOS/differential_robot/diffbot_ws; \
    source install/setup.bash; \
    sleep ${DEFAULT_SLEEP_TIME}; \
    ros2 launch diffbot_controller joystick_teleop.launch.py"

    gnome-terminal -- bash -c "$cmd; exec bash"
}

local_terminal_rviz() {
    cmd="source /opt/ros/humble/setup.bash; \
    cd /home/mario/REPOS/differential_robot/diffbot_ws; \
    source install/setup.bash; \
    sleep ${DEFAULT_SLEEP_TIME}; \
    rviz2 -d ./src/diffbot_description/rviz/simulated_robot.rviz"

    gnome-terminal -- bash -c "$cmd; exec bash"
}

# ==========================
# Main
# ==========================
main() {
    parse_command_line "$@"

    echo "Launching ROS2 environment..."
    echo "SSH target: $SSH_TARGET"

    remote_terminal
    remote_terminal_bno055_imu
    remote_terminal_rplidar
    local_terminal_joystick
    local_terminal_rviz
}

main "$@"
