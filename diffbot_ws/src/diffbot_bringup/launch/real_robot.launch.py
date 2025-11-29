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

    use_slam_arg = DeclareLaunchArgument(
        "use_slam",
        default_value="true"
    )

    use_slam = LaunchConfiguration("use_slam")

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
    
    bno055_imu = Node(
        package="bno055",
        executable="bno055",
        name="bno055_imu",
        output="screen",
        parameters=[os.path.join(get_package_share_directory("bno055"), "config", "bno055_params_i2c.yaml")]
    )

    local_localization = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("diffbot_localization"),
            "launch",
            "local_localization.launch.py"
        ),
    )

    laser_driver = Node(
        package="rplidar_ros",
        executable="rplidar_node",
        name="rplidar_node",
        parameters=[os.path.join(get_package_share_directory("diffbot_bringup"),"config","rplidar_a1.yaml")],
        output="screen"
    )

    localization = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("diffbot_localization"),
            "launch",
            "global_localization.launch.py"
        ),
        condition=UnlessCondition(use_slam)
    )

    slam = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("diffbot_mapping"),
            "launch",
            "slam.launch.py"
        ),
        condition=IfCondition(use_slam)
    )

    safety_stop = Node(
        package="diffbot_utils",
        executable="safety_stop",
        output="screen"
    )


    return LaunchDescription([
        use_joystick_arg,
        use_slam_arg,
        hardware_interface,
        controller,
        trajectory,
        joystick,
        bno055_imu,
        local_localization,
        laser_driver,
        safety_stop,
        localization,
        slam,
    ])