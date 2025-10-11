#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cex_pkg',
            executable='pub.py',
            name='talker',
            output='screen'
        ),
        Node(
            package='cex_pkg',
            executable='sub.py',
            name='listener',
            output='screen'
        )
    ])
