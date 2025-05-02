from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition

import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    use_python_arg = DeclareLaunchArgument(
        "use_python",
        default_value="False"
    )

    is_real_arg = DeclareLaunchArgument(
        "is_real",
        default_value="False"
    )

    imu_topic_arg = DeclareLaunchArgument(
        "imu_topic",
        default_value="imu/out"
    )

    use_python = LaunchConfiguration("use_python")
    is_real = LaunchConfiguration("is_real")
    imu_topic = LaunchConfiguration("imu_topic")

    static_transform_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["--x", "0.145", "--y", "0.0845", "--z", "0.1", "--qx", "0","--qy", "0","--qz", "0","--qw", "1", 
                   "--frame-id", "base_link_ekf", "--child-frame-id", "imu_link_ekf"]
    )

    robot_localization_sim = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[os.path.join(get_package_share_directory("diffbot_localization"),"config", "ekf.yaml")],
        condition=UnlessCondition(is_real)
    )

    robot_localization_real = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[os.path.join(get_package_share_directory("diffbot_localization"),"config", "ekf_real.yaml")],
        condition=IfCondition(is_real)
    )

    imu_republisher_py = Node(
        package="diffbot_localization",
        executable="imu_republisher_py",
        parameters=[{"imu_topic": imu_topic}],
        condition=IfCondition(use_python)
    )

    imu_republisher_cpp = Node(
        package="diffbot_localization",
        executable="imu_republisher",
        parameters=[{"imu_topic": imu_topic}],
        condition=UnlessCondition(use_python)
    )

    return LaunchDescription([
        use_python_arg,
        is_real_arg,
        imu_topic_arg,
        static_transform_publisher,
        robot_localization_sim,
        robot_localization_real,
        imu_republisher_py,
        imu_republisher_cpp
    ])
