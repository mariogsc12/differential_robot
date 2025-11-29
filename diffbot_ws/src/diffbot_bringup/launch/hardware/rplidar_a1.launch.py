import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    laser_driver = Node(
        package="rplidar_ros",
        executable="rplidar_node",
        name="rplidar_node",
        parameters=[os.path.join(get_package_share_directory("diffbot_bringup"),"config","rplidar_a1.yaml")],
        output="screen"
    )

    return LaunchDescription([
        laser_driver,
    ])