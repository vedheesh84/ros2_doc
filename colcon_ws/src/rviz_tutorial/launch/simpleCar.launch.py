#!/usr/bin/env python3
import os
from pathlib import Path
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_path
from launch.substitutions import Command
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
def generate_launch_description():
	path_to_xacro = get_package_share_path('rviz_tutorial') / 'urdf' / 'simple_robot_car.xacro'
	robot_description = ParameterValue(
		Command(['xacro ', str(path_to_xacro)]), value_type=str
	)

	joint_state_publisher_node = Node(
		package='joint_state_publisher_gui',
		executable='joint_state_publisher_gui',
		name='joint_state_publisher_gui',
		output='screen'
	)	

	robot_state_publisher_node = Node(
		package='robot_state_publisher',
		executable='robot_state_publisher',
		output='screen',
		parameters=[{
			'robot_description': robot_description
		}]
	)

	rviz_node = Node(
		package='rviz2',
		executable='rviz2',
		name='rviz2',
		output='screen'
	)

	return LaunchDescription([
		joint_state_publisher_node,
		robot_state_publisher_node,
		rviz_node
	])
