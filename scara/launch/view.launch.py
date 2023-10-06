from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import os
import xacro
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
    
    # Initialize Arguments
    description_file = LaunchConfiguration("description_file")

    # Get URDF via xacro
    pkg_path = os.path.join(get_package_share_directory('scara'))
    xacro_file = os.path.join(pkg_path,'urdf','urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {"robot_description": robot_description_config.toxml()}

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare('scara'), "rviz", "view.rviz"]
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="gui"
    )
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="rsp",
        output="both",
        parameters=[robot_description],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz",
        output="log",
        arguments=["-d", rviz_config_file],
        on_exit=Shutdown()
    )

    nodes = [
        joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node,
    ]

    return LaunchDescription(declared_arguments + nodes)