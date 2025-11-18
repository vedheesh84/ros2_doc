#!/usr/bin/env python3
"""
MASTER BRINGUP LAUNCH - Mobile Manipulator
===========================================

DESCRIPTION:
    Master launch file for mobile manipulator system
    Provides unified entry point with different operational modes

USAGE MODES:

    1. SIMULATION MODE (default):
       ros2 launch mobile_manipulator_bringup bringup.launch.py mode:=sim

    2. HARDWARE MODE:
       ros2 launch mobile_manipulator_bringup bringup.launch.py mode:=hardware

    3. SIMULATION + MOVEIT:
       ros2 launch mobile_manipulator_bringup bringup.launch.py mode:=sim_moveit

    4. HARDWARE + MOVEIT:
       ros2 launch mobile_manipulator_bringup bringup.launch.py mode:=hardware_moveit

ARGUMENTS:
    mode: Operational mode [sim|hardware|sim_moveit|hardware_moveit] (default: sim)
    rviz: Launch RViz (default: true)
    world: Gazebo world file for simulation (default: empty_world.world)
    serial_port: Arduino serial port for hardware (default: /dev/ttyACM0)
    baud_rate: Serial baud rate (default: 115200)

WHAT IT DOES:
    - Loads appropriate configuration based on mode
    - Launches simulation or hardware interface
    - Optionally launches MoveIt for motion planning
    - Provides RViz visualization
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition
import os

def launch_setup(context, *args, **kwargs):
    """Configure launch based on selected mode"""

    # Get mode from launch configuration
    mode = LaunchConfiguration('mode').perform(context)

    # Package paths
    pkg_share = FindPackageShare('mobile_manipulator_bringup')

    # Determine simulation vs hardware
    use_sim = 'sim' in mode
    use_moveit = 'moveit' in mode

    launch_actions = []

    # ========================================================================
    # SIMULATION MODE
    # ========================================================================
    if use_sim:
        # Launch Gazebo simulation
        gazebo_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([pkg_share, 'launch', 'sim', 'gazebo.launch.py'])
            ]),
            launch_arguments={
                'world': LaunchConfiguration('world'),
                'gui': LaunchConfiguration('gui'),
                'rviz': 'false' if use_moveit else LaunchConfiguration('rviz'),  # RViz from MoveIt if enabled
            }.items()
        )
        launch_actions.append(gazebo_launch)

    # ========================================================================
    # HARDWARE MODE
    # ========================================================================
    else:
        # Launch hardware interface
        hardware_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([pkg_share, 'launch', 'hardware', 'test_hardware.launch.py'])
            ]),
            launch_arguments={
                'serial_port': LaunchConfiguration('serial_port'),
                'baud_rate': LaunchConfiguration('baud_rate'),
                'launch_rviz': 'false' if use_moveit else LaunchConfiguration('rviz'),  # RViz from MoveIt if enabled
            }.items()
        )
        launch_actions.append(hardware_launch)

    # ========================================================================
    # MOVEIT MODE
    # ========================================================================
    if use_moveit:
        # Launch MoveIt motion planning
        moveit_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([pkg_share, 'launch', 'sim', 'moveit.launch.py'])
            ]),
            launch_arguments={
                'use_sim_time': 'true' if use_sim else 'false',
                'rviz': LaunchConfiguration('rviz'),
                'allow_trajectory_execution': 'true',
            }.items()
        )
        launch_actions.append(moveit_launch)

    return launch_actions

def generate_launch_description():

    # ========================================================================
    # LAUNCH ARGUMENTS
    # ========================================================================

    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='sim',
        description='Operational mode: sim, hardware, sim_moveit, hardware_moveit',
        choices=['sim', 'hardware', 'sim_moveit', 'hardware_moveit']
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz for visualization'
    )

    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Launch Gazebo GUI (simulation only)'
    )

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='empty_world.world',
        description='Gazebo world file (simulation only)'
    )

    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyACM0',
        description='Arduino serial port (hardware only)'
    )

    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value='115200',
        description='Serial baud rate (hardware only)'
    )

    # ========================================================================
    # LAUNCH DESCRIPTION
    # ========================================================================

    return LaunchDescription([
        # Arguments
        mode_arg,
        rviz_arg,
        gui_arg,
        world_arg,
        serial_port_arg,
        baud_rate_arg,

        # Launch setup based on mode
        OpaqueFunction(function=launch_setup)
    ])
