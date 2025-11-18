#!/usr/bin/env python3
"""
MOVEIT LAUNCH - Mobile Manipulator
===================================

DESCRIPTION:
    Launch MoveIt2 motion planning for the robotic arm
    Can work with either simulation or real hardware

USAGE:
    # With Gazebo simulation
    ros2 launch mobile_manipulator_bringup moveit.launch.py use_sim_time:=true

    # With real hardware
    ros2 launch mobile_manipulator_bringup moveit.launch.py use_sim_time:=false

ARGUMENTS:
    use_sim_time: Use simulation time (default: true)
    rviz: Launch RViz with MoveIt plugin (default: true)
    allow_trajectory_execution: Allow executing planned trajectories (default: true)

WHAT IT LAUNCHES:
    1. move_group node (motion planning server)
    2. RViz with MoveIt motion planning plugin
    3. Robot state publisher
    4. Trajectory execution manager
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
import yaml

def load_yaml(package_name, file_path):
    """Load a YAML file and return as dictionary"""
    package_path = FindPackageShare(package_name).find(package_name)
    absolute_file_path = PathJoinSubstitution([package_path, file_path])

    try:
        with open(absolute_file_path.perform(None), 'r') as file:
            return yaml.safe_load(file)
    except Exception as e:
        return {}

def generate_launch_description():

    # Package paths
    pkg_share = FindPackageShare('mobile_manipulator_bringup')

    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz with MoveIt plugin'
    )

    allow_trajectory_execution_arg = DeclareLaunchArgument(
        'allow_trajectory_execution',
        default_value='true',
        description='Allow executing planned trajectories'
    )

    # File paths
    urdf_file = PathJoinSubstitution([
        pkg_share, 'urdf', 'mobile_manipulator.urdf.xacro'
    ])

    srdf_file = PathJoinSubstitution([
        pkg_share, 'config', 'arm', 'mobile_manipulator.srdf'
    ])

    moveit_controllers_file = PathJoinSubstitution([
        pkg_share, 'config', 'arm', 'moveit_controllers.yaml'
    ])

    kinematics_file = PathJoinSubstitution([
        pkg_share, 'config', 'arm', 'kinematics.yaml'
    ])

    joint_limits_file = PathJoinSubstitution([
        pkg_share, 'config', 'arm', 'joint_limits.yaml'
    ])

    rviz_config = PathJoinSubstitution([
        pkg_share, 'config', 'moveit', 'moveit.rviz'
    ])

    # Process URDF
    robot_description_content = Command([
        'xacro ', urdf_file,
        ' use_sim:=', LaunchConfiguration('use_sim_time')
    ])

    robot_description = {'robot_description': robot_description_content}

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

    # MoveIt configuration parameters
    robot_description_semantic = {
        'robot_description_semantic': Command(['cat ', srdf_file])
    }

    kinematics_yaml = {
        'robot_description_kinematics': Command(['cat ', kinematics_file])
    }

    joint_limits_yaml = {
        'robot_description_planning': Command(['cat ', joint_limits_file])
    }

    # Planning pipeline parameters
    planning_pipelines_config = {
        'planning_pipelines': ['ompl'],
        'ompl': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': 'default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints',
            'start_state_max_bounds_error': 0.1,
        }
    }

    # Trajectory execution parameters
    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }

    # Planning scene monitor parameters
    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }

    # move_group node
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        name='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            joint_limits_yaml,
            planning_pipelines_config,
            trajectory_execution,
            planning_scene_monitor_parameters,
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'allow_trajectory_execution': LaunchConfiguration('allow_trajectory_execution')},
        ],
    )

    # RViz with MoveIt plugin
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            joint_limits_yaml,
            planning_pipelines_config,
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    return LaunchDescription([
        # Arguments
        use_sim_time_arg,
        rviz_arg,
        allow_trajectory_execution_arg,

        # Nodes
        robot_state_publisher,
        move_group_node,
        rviz_node,
    ])
