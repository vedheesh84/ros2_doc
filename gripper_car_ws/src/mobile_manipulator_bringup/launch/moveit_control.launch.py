#!/usr/bin/env python3
"""
MoveIt Control Launch File for Mobile Manipulator

This launch file starts MoveIt2 for arm motion planning and control.
It should be launched AFTER the hardware_bringup.launch.py.

Usage:
  # First start hardware:
  ros2 launch mobile_manipulator_bringup hardware_bringup.launch.py

  # Then start MoveIt:
  ros2 launch mobile_manipulator_bringup moveit_control.launch.py

  # Or run demo mode:
  ros2 launch mobile_manipulator_bringup moveit_control.launch.py demo:=true

Author: Mobile Manipulator Project
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Package directories
    pkg_mobile_arm_config = FindPackageShare('mobile_arm_manipulator_config')
    pkg_mobile_manipulator = FindPackageShare('mobile_manipulator_bringup')

    # Launch arguments
    demo_arg = DeclareLaunchArgument(
        'demo',
        default_value='false',
        description='Run arm controller demo sequence'
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Start RViz with MoveIt'
    )

    # Get configurations
    demo = LaunchConfiguration('demo')
    use_rviz = LaunchConfiguration('use_rviz')

    # ========================================================================
    # NODES
    # ========================================================================

    # MoveIt Arm Controller
    moveit_controller_node = Node(
        package='mobile_manipulator_bringup',
        executable='moveit_arm_controller.py',
        name='moveit_arm_controller',
        output='screen',
        parameters=[{
            'use_sim_time': False
        }],
        arguments=['--demo'] if demo else [],
    )

    # Include MoveIt move_group launch from mobile_arm_manipulator_config
    # Note: This assumes the MoveIt config package has a move_group.launch.py
    moveit_move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                pkg_mobile_arm_config,
                'launch',
                'move_group.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': 'false',
        }.items(),
        condition=IfCondition('true')  # Always include if available
    )

    # MoveIt RViz (optional)
    moveit_rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                pkg_mobile_arm_config,
                'launch',
                'moveit_rviz.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': 'false',
        }.items(),
        condition=IfCondition(use_rviz)
    )

    # ========================================================================
    # LAUNCH DESCRIPTION
    # ========================================================================

    return LaunchDescription([
        # Arguments
        demo_arg,
        use_rviz_arg,

        # Nodes and includes
        moveit_controller_node,
        # Uncomment these if MoveIt config launch files exist:
        # moveit_move_group_launch,
        # moveit_rviz_launch,
    ])
