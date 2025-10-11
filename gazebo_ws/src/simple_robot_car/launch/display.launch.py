import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command, PathJoinSubstitution

def generate_launch_description():
    pkg_simple = get_package_share_directory('simple_robot_car')
    pkg_gazebo = get_package_share_directory('gazebo_ros')

    urdf_file = os.path.join(pkg_simple, 'urdf', 'simple_robot_car.urdf')

    return LaunchDescription([

        # Start Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo, 'launch', 'gazebo.launch.py')
              )
          ),

        # Publish robot_description
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': open(urdf_file).read()}]
        ), 

        # call rviz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        ),

        # Spawn in Gazebo
        Node(
            package='gazebo_ros',
             executable='spawn_entity.py',
             arguments=['-topic', 'robot_description', '-entity', 'simple_robot_car'],
            output='screen'
        )

    ])