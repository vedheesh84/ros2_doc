#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    #rviz config and robot description paths
    rviz_package_name = 'rviz_tutorial'
    rviz_pkg_share_dir = get_package_share_directory(rviz_package_name)
    rviz_config_file = os.path.join(rviz_pkg_share_dir, 'config', 'map.rviz')


    # Robot URDF file path
    urdf_file_name = 'robot.urdf.xacro'
    urdf_path = os.path.join(rviz_pkg_share_dir, 'urdf', urdf_file_name)


    # Robot Description parameter
    robot_description_content = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)
    robot_description_param = {'robot_description': robot_description_content}


	# Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description_param, {'use_sim_time': use_sim_time}]
    )


    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )
	
    # Path to the SLAM Toolbox localization launch file
    slam_launch_file = os.path.join(
        get_package_share_directory('slam_toolbox'),
        'launch',
        'localization_launch.py'
    )

    # Path to your params file (inside your own package)
    params_file = os.path.join(
        get_package_share_directory('rviz_tutorial'),
        'config',
        'localisation_params.yaml'
    )

    # Include the localization launch description
    slam_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch_file),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': params_file
        }.items(),
    )

   
    ld = LaunchDescription()

    
    ld.add_action(robot_state_publisher_node)
    ld.add_action(rviz_node)
    ld.add_action(slam_localization)    

    return ld