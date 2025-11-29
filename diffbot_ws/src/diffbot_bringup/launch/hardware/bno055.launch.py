import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    bno055_imu = Node(
        package="bno055",
        executable="bno055",
        name="bno055",
        output="screen",
        parameters=[os.path.join(get_package_share_directory("bno055"), "config", "bno055_params_i2c.yaml")]
    )

    return LaunchDescription([
        bno055_imu,
    ])