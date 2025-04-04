import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command

def generate_launch_description():
    diffbot_description_dir = get_package_share_directory("diffbot_description")

    robot_description = ParameterValue(
        Command([
            "xacro ", 
            os.path.join(get_package_share_directory("diffbot_description"),
                        "urdf", 
                        "diffbot.urdf.xacro"
            ),
            " is_sim:=False",
        ]),
        value_type=str
    )
    
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}]
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description,
             "use_sim_time": False},
            os.path.join(
                get_package_share_directory("diffbot_controller"),
                "config",
                "diffbot_controllers.yaml"
            )
        ]
    )

    return LaunchDescription([
        robot_state_publisher,
        controller_manager
    ])