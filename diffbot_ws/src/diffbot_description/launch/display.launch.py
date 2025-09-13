import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import UnlessCondition, IfCondition

def generate_launch_description():
    diffbot_description_dir = get_package_share_directory("diffbot_description")

    use_gui_arg = DeclareLaunchArgument(
        "use_gui",
        default_value="False",
    )
    
    use_gui = LaunchConfiguration("use_gui")

    model_arg = DeclareLaunchArgument(name="model", default_value=os.path.join(
                                        diffbot_description_dir, "urdf", "diffbot.urdf.xacro"
                                        ),
                                      description="Absolute path to robot urdf file")

    robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration("model")]),
                                       value_type=str)

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}]
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        condition=IfCondition(use_gui)
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(diffbot_description_dir, "rviz", "display.rviz")],
    )

    return LaunchDescription([
        use_gui_arg,
        model_arg,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node
    ])