from launch import LaunchDescription
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[os.path.join(get_package_share_directory("diffbot_localization"),"config","ekf.yaml")]
    )

    return LaunchDescription([
        localization_node
    ])