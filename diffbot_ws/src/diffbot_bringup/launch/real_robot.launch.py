import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():

    use_joystick = LaunchConfiguration("use_joystick")

    hardware_interface = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("diffbot_firmware"),
            "launch",
            "hardware_interface.launch.py"
        ),
    )

    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("diffbot_controller"),
            "launch",
            "controller.launch.py"
        ),
        launch_arguments={
            "use_simple_controller": "False",
            "use_python": "False"
        }.items(),
    )

    joystick = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("diffbot_controller"),
            "launch",
            "joystick_teleop.launch.py"
        ),
        condition=IfCondition(use_joystick),
        launch_arguments={
            "use_sim_time": "False"
        }.items()
    )

    local_localization = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("diffbot_localization"),
            "launch",
            "local_localization.launch.py"
        ),
        launch_arguments={
            "imu_topic": "imu_data"
        }.items()
    )

    return LaunchDescription([
        hardware_interface,
        controller,
        joystick,
        local_localization
    ])