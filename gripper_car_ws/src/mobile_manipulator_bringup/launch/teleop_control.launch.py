#!/usr/bin/env python3
"""
Teleop Control Launch File

Launches keyboard teleoperation for mobile base control.
The arm can be controlled via the integrated teleop or separately via MoveIt.

Usage:
  ros2 launch mobile_manipulator_bringup teleop_control.launch.py

  # With integrated teleop (base + arm):
  ros2 launch mobile_manipulator_bringup teleop_control.launch.py mode:=integrated

  # Base only:
  ros2 launch mobile_manipulator_bringup teleop_control.launch.py mode:=base_only

Author: Mobile Manipulator Project
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node


def generate_launch_description():
    # Launch arguments
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='integrated',
        description='Teleop mode: integrated, base_only'
    )

    speed_linear_arg = DeclareLaunchArgument(
        'speed_linear',
        default_value='0.5',
        description='Maximum linear velocity (m/s)'
    )

    speed_angular_arg = DeclareLaunchArgument(
        'speed_angular',
        default_value='1.0',
        description='Maximum angular velocity (rad/s)'
    )

    # Get configurations
    mode = LaunchConfiguration('mode')
    speed_linear = LaunchConfiguration('speed_linear')
    speed_angular = LaunchConfiguration('speed_angular')

    # Check if mode is integrated
    is_integrated = LaunchConfiguration('mode', default='integrated')

    # ========================================================================
    # NODES
    # ========================================================================

    # Integrated teleop (base + arm control)
    integrated_teleop_node = Node(
        package='mobile_manipulator_bringup',
        executable='integrated_teleop_control.py',
        name='integrated_teleop_control',
        output='screen',
        prefix='xterm -e',  # Run in separate terminal for keyboard input
        emulate_tty=True,
    )

    # Base-only teleop (standard teleop_twist_keyboard)
    base_teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        prefix='xterm -e',
        remappings=[
            ('/cmd_vel', '/cmd_vel')
        ],
        parameters=[{
            'speed': speed_linear,
            'turn': speed_angular,
        }],
    )

    # ========================================================================
    # LAUNCH DESCRIPTION
    # ========================================================================

    return LaunchDescription([
        # Arguments
        mode_arg,
        speed_linear_arg,
        speed_angular_arg,

        # Choose teleop mode
        integrated_teleop_node,  # Always launch integrated by default
        # To use base_only, comment out integrated and uncomment base_teleop_node
        # base_teleop_node,
    ])
