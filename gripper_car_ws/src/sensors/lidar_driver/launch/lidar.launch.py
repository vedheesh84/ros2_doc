#!/usr/bin/env python3
"""
LiDAR Driver Launch File
=========================
Usage:
    ros2 launch lidar_driver lidar.launch.py
    ros2 launch lidar_driver lidar.launch.py lidar_type:=rplidar
    ros2 launch lidar_driver lidar.launch.py lidar_type:=ydlidar lidar_port:=/dev/ttyUSB0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    lidar_type = LaunchConfiguration('lidar_type').perform(context)
    lidar_port = LaunchConfiguration('lidar_port').perform(context)
    lidar_frame = LaunchConfiguration('lidar_frame').perform(context)

    nodes = []

    if lidar_type.lower() == 'rplidar':
        nodes.append(Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            parameters=[{
                'serial_port': lidar_port,
                'serial_baudrate': 115200,
                'frame_id': lidar_frame,
                'inverted': False,
                'angle_compensate': True,
                'scan_mode': 'Standard',
            }],
            output='screen'
        ))
    elif lidar_type.lower() == 'ydlidar':
        nodes.append(Node(
            package='ydlidar_ros2_driver',
            executable='ydlidar_ros2_driver_node',
            name='ydlidar_node',
            parameters=[{
                'port': lidar_port,
                'baudrate': 128000,
                'frame_id': lidar_frame,
                'angle_min': -180.0,
                'angle_max': 180.0,
                'range_min': 0.1,
                'range_max': 12.0,
            }],
            output='screen'
        ))
    else:
        nodes.append(Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            parameters=[{
                'serial_port': lidar_port,
                'serial_baudrate': 115200,
                'frame_id': lidar_frame,
            }],
            output='screen'
        ))

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('lidar_type', default_value='rplidar'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('lidar_frame', default_value='laser_frame'),
        OpaqueFunction(function=launch_setup),
    ])
