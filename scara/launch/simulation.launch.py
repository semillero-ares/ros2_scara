import os
import xacro

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription, 
    Shutdown, 
    DeclareLaunchArgument 
    )
from launch.conditions import IfCondition
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # Set ROBOT : get URDF via xacro
    pkg_path = os.path.join(get_package_share_directory('scara'))
    xacro_file = os.path.join(pkg_path,'urdf','gazebo.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {"robot_description": robot_description_config.toxml()}
    
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # RViz
    rviz_config_file = PathJoinSubstitution([FindPackageShare("scara"), 
        "rviz", "view.rviz"])
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz",
        output="log",
        arguments=["-d", rviz_config_file],
        on_exit=Shutdown()
    )

    # Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=[
            "--frame-id","world", 
            "--child-frame-id","base_link"],
    )

    # gazebo_ros2_control
    scara_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "scara_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        on_exit=[scara_controller_spawner]
    )

    # Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
        get_package_share_directory('gazebo_ros'), 'launch'), "/gazebo.launch.py"]),
    )
    gazebo_robot_spawner = Node(
        package="gazebo_ros", 
        executable="spawn_entity.py",
        arguments=[
            "-topic", "robot_description",
            "-entity", "scara",
            "-timeout", "120"],
        output="screen",
        on_exit=[joint_state_broadcaster_spawner]
    )

    nodes = [
        gazebo,
        robot_state_publisher_node,
        rviz_node,
        static_tf,
        gazebo_robot_spawner,
    ]

    return LaunchDescription(nodes)
