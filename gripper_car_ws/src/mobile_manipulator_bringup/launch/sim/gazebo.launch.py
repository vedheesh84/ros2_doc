#!/usr/bin/env python3
"""
GAZEBO SIMULATION LAUNCH - Mobile Manipulator
==============================================

DESCRIPTION:
    Launch complete robot in Gazebo simulation
    Includes robot spawning, controllers, and RViz

USAGE:
    ros2 launch mobile_manipulator_bringup gazebo.launch.py

ARGUMENTS:
    world: Gazebo world file (default: empty_world.world)
    use_sim_time: Use simulation time (default: true)
    gui: Launch Gazebo GUI (default: true)
    rviz: Launch RViz (default: true)

WHAT IT LAUNCHES:
    1. Gazebo simulator
    2. Robot spawner
    3. Robot state publisher
    4. Controller spawner
    5. RViz (optional)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
import os

def generate_launch_description():

    # Package paths
    pkg_share = FindPackageShare('mobile_manipulator_bringup')
    gazebo_ros_share = FindPackageShare('gazebo_ros')

    # Launch arguments
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='empty_world.world',
        description='Gazebo world file name'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Launch Gazebo GUI'
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz'
    )

    # File paths
    urdf_file = PathJoinSubstitution([
        pkg_share, 'urdf', 'mobile_manipulator.urdf.xacro'
    ])

    world_file = PathJoinSubstitution([
        pkg_share, 'worlds', LaunchConfiguration('world')
    ])

    rviz_config = PathJoinSubstitution([
        pkg_share, 'config', 'gazebo', 'mobile_manipulator.rviz'
    ])

    controller_config = PathJoinSubstitution([
        pkg_share, 'config', 'arm', 'controllers.yaml'
    ])

    # Process URDF
    robot_description_content = Command([
        'xacro ', urdf_file,
        ' use_sim:=true',
        ' hardware_interface:=gazebo_ros2_control/GazeboSystem'
    ])

    robot_description = {'robot_description': robot_description_content}

    # Gazebo server
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([gazebo_ros_share, 'launch', 'gzserver.launch.py'])
        ]),
        launch_arguments={
            'world': world_file,
            'verbose': 'false',
        }.items()
    )

    # Gazebo client (GUI)
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([gazebo_ros_share, 'launch', 'gzclient.launch.py'])
        ]),
        condition=IfCondition(LaunchConfiguration('gui'))
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            robot_description,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )

    # Spawn robot in Gazebo
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'mobile_manipulator',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.2',
        ],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # Controller manager (loaded by Gazebo plugin, this is for spawning controllers)
    load_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        name='joint_state_broadcaster_spawner',
        output='screen',
        arguments=['joint_state_broadcaster'],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    load_arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        name='arm_controller_spawner',
        output='screen',
        arguments=['arm_controller'],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    load_gripper_controller = Node(
        package='controller_manager',
        executable='spawner',
        name='gripper_controller_spawner',
        output='screen',
        arguments=['gripper_controller'],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # Delay controller spawning after robot spawns
    delay_joint_state_broadcaster_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=[load_joint_state_broadcaster],
        )
    )

    delay_arm_controller_after_joint_state = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_joint_state_broadcaster,
            on_exit=[load_arm_controller],
        )
    )

    delay_gripper_controller_after_arm = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_arm_controller,
            on_exit=[load_gripper_controller],
        )
    )

    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    return LaunchDescription([
        # Arguments
        world_arg,
        use_sim_time_arg,
        gui_arg,
        rviz_arg,

        # Gazebo
        gzserver,
        gzclient,

        # Robot
        robot_state_publisher,
        spawn_robot,

        # Controllers (with delays)
        delay_joint_state_broadcaster_after_spawn,
        delay_arm_controller_after_joint_state,
        delay_gripper_controller_after_arm,

        # Visualization
        rviz,
    ])
