#!/usr/bin/env python3
"""
HARDWARE TEST LAUNCH - Mobile Manipulator
==========================================

DESCRIPTION:
    Quick test launch for hardware bridge
    Tests Arduino communication and basic functionality

USAGE:
    ros2 launch mobile_manipulator_bringup test_hardware.launch.py

ARGUMENTS:
    serial_port: Arduino serial port (default: /dev/ttyACM0)
    baud_rate: Serial baud rate (default: 115200)

WHAT IT DOES:
    1. Launches unified hardware bridge
    2. Publishes robot state
    3. Allows testing via /cmd_vel and /arm/joint_commands topics
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
import os

def generate_launch_description():

    # Package path
    pkg_share = FindPackageShare('mobile_manipulator_bringup')

    # Launch arguments
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyACM0',
        description='Arduino serial port'
    )

    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value='115200',
        description='Serial baud rate'
    )

    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz',
        default_value='true',
        description='Launch RViz for visualization'
    )

    # Get URDF file
    urdf_file = PathJoinSubstitution([
        pkg_share, 'urdf', 'mobile_manipulator.urdf.xacro'
    ])

    # Unified hardware bridge node
    hardware_bridge_node = Node(
        package='mobile_manipulator_bringup',
        executable='unified_hardware_bridge.py',
        name='unified_hardware_bridge',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'baud_rate': LaunchConfiguration('baud_rate'),
            'wheel_radius': 0.075,
            'wheel_base': 0.67,
            'encoder_ticks_per_rev': 360,
            'publish_rate': 50.0,
        }],
        emulate_tty=True,
    )

    # Robot state publisher (for TF and joint states)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ExecuteProcess(
                cmd=['xacro', urdf_file, 'use_sim:=false'],
                output='screen'
            ).output
        }],
    )

    # RViz (optional)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(LaunchConfiguration('launch_rviz')),
        arguments=['-d', PathJoinSubstitution([
            pkg_share, 'config', 'gazebo', 'mobile_manipulator.rviz'
        ])],
    )

    return LaunchDescription([
        # Arguments
        serial_port_arg,
        baud_rate_arg,
        launch_rviz_arg,

        # Nodes
        hardware_bridge_node,
        robot_state_publisher_node,
        rviz_node,
    ])
