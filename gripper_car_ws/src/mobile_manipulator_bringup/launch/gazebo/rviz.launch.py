#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    pkg_share = FindPackageShare("mobile_manipulator_bringup").find("mobile_manipulator_bringup")

 

    # Generate robot_description from xacro
    robot_description_xacro = PathJoinSubstitution([
        pkg_share,
        'urdf',
        'mobile_arm.urdf.xacro'
    ])

    robot_description = ParameterValue(
        Command(['xacro ', robot_description_xacro]),
        value_type=str)
    

    # RViz configuration file (optional)
    rviz_config_path = os.path.join(pkg_share, "rviz", "mobile_manipulator.rviz")

    # -----------------------------------------------------------
    # Nodes
    # -----------------------------------------------------------

    # Joint state publisher (GUI sliders)
    joint_state_pub_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui"
    )

    # Robot state publisher (publishes TF tree)
    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{'robot_description': robot_description}]
    )

    # RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_path] if os.path.exists(rviz_config_path) else []
    )

    return LaunchDescription([
        joint_state_pub_gui,
        robot_state_pub,
        rviz_node
    ])
