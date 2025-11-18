#!/usr/bin/env python3
"""
Hardware Bringup Launch File for Mobile Manipulator

This launch file starts all necessary nodes for hardware operation:
- Unified hardware bridge (Arduino communication)
- Robot state publisher (URDF/TF)
- Joint state publisher
- Optional: RViz visualization

Usage:
  ros2 launch mobile_manipulator_bringup hardware_bringup.launch.py

Optional arguments:
  serial_port:=/dev/ttyACM0
  use_rviz:=true
  rviz_config:=path/to/config.rviz

Author: Mobile Manipulator Project
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os


def generate_launch_description():
    # Package directories
    pkg_mobile_manipulator = FindPackageShare('mobile_manipulator_bringup')

    # Launch arguments
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyACM0',
        description='Serial port for Arduino Mega'
    )

    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value='115200',
        description='Serial baud rate'
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Start RViz for visualization'
    )

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=PathJoinSubstitution([
            pkg_mobile_manipulator,
            'config',
            'hardware',
            'mobile_manipulator.rviz'
        ]),
        description='RViz configuration file'
    )

    wheel_radius_arg = DeclareLaunchArgument(
        'wheel_radius',
        default_value='0.075',
        description='Wheel radius in meters'
    )

    wheel_base_arg = DeclareLaunchArgument(
        'wheel_base',
        default_value='0.67',
        description='Wheel base (left-right separation) in meters'
    )

    encoder_ticks_arg = DeclareLaunchArgument(
        'encoder_ticks_per_rev',
        default_value='360',
        description='Encoder ticks per revolution'
    )

    # Get launch configurations
    serial_port = LaunchConfiguration('serial_port')
    baud_rate = LaunchConfiguration('baud_rate')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    wheel_radius = LaunchConfiguration('wheel_radius')
    wheel_base = LaunchConfiguration('wheel_base')
    encoder_ticks = LaunchConfiguration('encoder_ticks_per_rev')

    # ========================================================================
    # NODES
    # ========================================================================

    # 1. Unified Hardware Bridge Node
    hardware_bridge_node = Node(
        package='mobile_manipulator_bringup',
        executable='unified_hardware_bridge.py',
        name='unified_hardware_bridge',
        output='screen',
        parameters=[{
            'serial_port': serial_port,
            'baud_rate': baud_rate,
            'wheel_radius': wheel_radius,
            'wheel_base': wheel_base,
            'encoder_ticks_per_rev': encoder_ticks,
            'publish_rate': 50.0,
            'cmd_timeout': 0.5,
            'reconnect_delay': 2.0,
        }],
        emulate_tty=True,
    )

    # 2. Robot State Publisher
    # Load URDF
    urdf_path = PathJoinSubstitution([
        pkg_mobile_manipulator,
        'urdf',
        'mobile_manipulator.urdf.xacro'
    ])

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': urdf_path,
            'use_sim_time': False
        }]
    )

    # 3. Joint State Publisher (GUI optional)
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'source_list': ['/joint_states'],  # Listen to hardware bridge
        }]
    )

    # 4. RViz (optional)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        condition=IfCondition(use_rviz),
        parameters=[{'use_sim_time': False}]
    )

    # ========================================================================
    # LAUNCH DESCRIPTION
    # ========================================================================

    return LaunchDescription([
        # Arguments
        serial_port_arg,
        baud_rate_arg,
        use_rviz_arg,
        rviz_config_arg,
        wheel_radius_arg,
        wheel_base_arg,
        encoder_ticks_arg,

        # Nodes
        hardware_bridge_node,
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node,
    ])
