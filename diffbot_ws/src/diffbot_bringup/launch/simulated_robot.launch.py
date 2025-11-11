import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import UnlessCondition, IfCondition
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_slam_arg = DeclareLaunchArgument(
        "use_slam",
        default_value="false"
    )
    use_slam = LaunchConfiguration("use_slam")

    gazebo = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("diffbot_description"),
            "launch",
            "gazebo.launch.py"
        )
    )

    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("diffbot_controller"),
            "launch",
            "controller.launch.py"
        ),
        launch_arguments={
            "use_simple_controller": "False",
            "use_python": "False",
            "use_sim_time": "True",
        }.items()
    )

    joystick = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("diffbot_controller"),
            "launch",
            "joystick_teleop.launch.py"
        )
    )

    trajectory = Node(
        package="diffbot_utils",
        executable="trajectory_drawer"
    )

    rviz_localization = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", os.path.join(get_package_share_directory("diffbot_description"),"rviz","localization.rviz")],
        parameters=[{"use_sim_time": True}],
        condition=UnlessCondition(use_slam)
    )

    rviz_slam = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", os.path.join(get_package_share_directory("diffbot_description"),"rviz","slam.rviz")],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(use_slam)
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
        use_slam_arg,
        gazebo,
        controller,
        joystick,
        trajectory,
        rviz_localization,
        rviz_slam,
        localization,
        slam,
        safety_stop
    ])