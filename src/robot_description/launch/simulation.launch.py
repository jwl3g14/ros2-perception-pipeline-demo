#!/usr/bin/env python3
"""
Launch Gazebo simulation with robot arm and shelf world.

Usage:
  ros2 launch robot_description simulation.launch.py
  ros2 launch robot_description simulation.launch.py gui:=false  # headless
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Package paths
    pkg_robot_description = get_package_share_directory('robot_description')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # File paths
    urdf_file = os.path.join(pkg_robot_description, 'urdf', 'robot.urdf.xacro')
    world_file = os.path.join(pkg_robot_description, 'worlds', 'shelf.world')

    # Launch arguments
    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Start Gazebo with GUI'
    )

    # Robot description from xacro
    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )

    # Gazebo server
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={
            'world': world_file,
            'verbose': 'true',
        }.items()
    )

    # Gazebo client (GUI)
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('gui'))
    )

    # Robot state publisher (publishes TF from URDF)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # Spawn robot in Gazebo
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'arm_robot',
            '-x', '0',
            '-y', '0',
            '-z', '0',
        ],
        output='screen'
    )

    # Spawn joint_state_broadcaster (after robot is spawned)
    spawn_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
        output='screen'
    )

    # Spawn joint_trajectory_controller (after joint_state_broadcaster)
    spawn_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_trajectory_controller'],
        output='screen'
    )

    # Chain controller spawning after robot spawn
    spawn_controllers_after_robot = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=[spawn_joint_state_broadcaster],
        )
    )

    spawn_trajectory_after_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_joint_state_broadcaster,
            on_exit=[spawn_joint_trajectory_controller],
        )
    )

    return LaunchDescription([
        gui_arg,
        gazebo_server,
        gazebo_client,
        robot_state_publisher,
        spawn_robot,
        spawn_controllers_after_robot,
        spawn_trajectory_after_broadcaster,
    ])
