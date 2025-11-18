#!/usr/bin/env python3
"""
Complete Mobile Manipulator System Launch File

This master launch file brings up the entire mobile manipulator system:
- Hardware bridge (Arduino communication)
- Robot description and visualization
- Teleop control
- MoveIt arm control (optional)
- Navigation stack (optional)

Usage:
  # Full system with all features:
  ros2 launch mobile_manipulator_bringup complete_system.launch.py

  # Hardware + teleop only:
  ros2 launch mobile_manipulator_bringup complete_system.launch.py \
    use_moveit:=false use_nav:=false

  # With custom serial port:
  ros2 launch mobile_manipulator_bringup complete_system.launch.py \
    serial_port:=/dev/ttyUSB0

Author: Mobile Manipulator Project
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    GroupAction
)
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    Command,
    FindExecutable
)
from launch.conditions import IfCondition
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os


def generate_launch_description():
    # ========================================================================
    # PACKAGE DIRECTORIES
    # ========================================================================
    pkg_mobile_manipulator = FindPackageShare('mobile_manipulator_bringup')
    pkg_mobile_arm_config = FindPackageShare('mobile_arm_manipulator_config')

    # ========================================================================
    # LAUNCH ARGUMENTS
    # ========================================================================

    # Hardware arguments
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyACM0',
        description='Arduino serial port'
    )

    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value='115200',
        description='Serial communication baud rate'
    )

    # Robot parameters
    wheel_radius_arg = DeclareLaunchArgument(
        'wheel_radius',
        default_value='0.075',
        description='Wheel radius in meters'
    )

    wheel_base_arg = DeclareLaunchArgument(
        'wheel_base',
        default_value='0.67',
        description='Distance between left and right wheels (meters)'
    )

    encoder_ticks_arg = DeclareLaunchArgument(
        'encoder_ticks_per_rev',
        default_value='360',
        description='Encoder ticks per wheel revolution'
    )

    # Feature toggles
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz visualization'
    )

    use_moveit_arg = DeclareLaunchArgument(
        'use_moveit',
        default_value='true',
        description='Launch MoveIt for arm control'
    )

    use_teleop_arg = DeclareLaunchArgument(
        'use_teleop',
        default_value='true',
        description='Launch keyboard teleop'
    )

    use_nav_arg = DeclareLaunchArgument(
        'use_nav',
        default_value='false',
        description='Launch navigation stack (Nav2)'
    )

    # Configuration files
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

    # Get launch configurations
    serial_port = LaunchConfiguration('serial_port')
    baud_rate = LaunchConfiguration('baud_rate')
    wheel_radius = LaunchConfiguration('wheel_radius')
    wheel_base = LaunchConfiguration('wheel_base')
    encoder_ticks = LaunchConfiguration('encoder_ticks_per_rev')
    use_rviz = LaunchConfiguration('use_rviz')
    use_moveit = LaunchConfiguration('use_moveit')
    use_teleop = LaunchConfiguration('use_teleop')
    use_nav = LaunchConfiguration('use_nav')
    rviz_config = LaunchConfiguration('rviz_config')

    # ========================================================================
    # ROBOT DESCRIPTION
    # ========================================================================

    # Get URDF via xacro
    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ',
        PathJoinSubstitution([
            pkg_mobile_manipulator,
            'urdf',
            'mobile_manipulator.urdf.xacro'
        ])
    ])

    robot_description = {'robot_description': robot_description_content}

    # ========================================================================
    # CORE NODES
    # ========================================================================

    # 1. Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            robot_description,
            {'use_sim_time': False}
        ]
    )

    # 2. Joint State Publisher
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'source_list': ['/joint_states']
        }]
    )

    # 3. Unified Hardware Bridge
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
        emulate_tty=True
    )

    # ========================================================================
    # OPTIONAL NODES
    # ========================================================================

    # 4. RViz Visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': False}],
        condition=IfCondition(use_rviz)
    )

    # 5. Integrated Teleop Control
    teleop_node = Node(
        package='mobile_manipulator_bringup',
        executable='integrated_teleop_control.py',
        name='integrated_teleop_control',
        output='screen',
        prefix='xterm -e',
        emulate_tty=True,
        condition=IfCondition(use_teleop)
    )

    # 6. MoveIt Arm Controller
    moveit_controller_node = Node(
        package='mobile_manipulator_bringup',
        executable='moveit_arm_controller.py',
        name='moveit_arm_controller',
        output='screen',
        parameters=[{'use_sim_time': False}],
        condition=IfCondition(use_moveit)
    )

    # ========================================================================
    # LAUNCH DESCRIPTION
    # ========================================================================

    return LaunchDescription([
        # ====== ARGUMENTS ======
        serial_port_arg,
        baud_rate_arg,
        wheel_radius_arg,
        wheel_base_arg,
        encoder_ticks_arg,
        use_rviz_arg,
        use_moveit_arg,
        use_teleop_arg,
        use_nav_arg,
        rviz_config_arg,

        # ====== CORE NODES (Always launched) ======
        robot_state_publisher_node,
        joint_state_publisher_node,
        hardware_bridge_node,

        # ====== OPTIONAL NODES ======
        rviz_node,
        teleop_node,
        moveit_controller_node,

        # Note: Navigation stack would be included here if use_nav:=true
        # This would typically include Nav2 bringup
    ])
