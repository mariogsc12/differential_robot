import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():

    use_joystick_arg = DeclareLaunchArgument(
        "use_joystick",
        default_value="False"
    )
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

    trajectory = Node(
        package="diffbot_utils",
        executable="trajectory_drawer"
    )
    
#    witmotion_imu = IncludeLaunchDescription(
#        os.path.join(
#            get_package_share_directory("witmotion_ros"),
#            "launch",
#            "wt61c.launch.py"
#        ),
#    )

    local_localization = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("diffbot_localization"),
            "launch",
            "local_localization.launch.py"
        ),
    )


    return LaunchDescription([
        use_joystick_arg,
        hardware_interface,
        controller,
        trajectory,
        joystick,
        #witmotion_imu,
        local_localization
    ])