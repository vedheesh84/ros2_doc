#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

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

    return LaunchDescription([slam_localization])
