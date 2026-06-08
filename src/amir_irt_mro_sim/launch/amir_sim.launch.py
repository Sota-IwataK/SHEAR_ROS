#!/usr/bin/env python3
"""
シミュレーション用統合launchファイル
Ignition Gazebo + amir_irt_mro 全ノードを起動
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    deadman_required = LaunchConfiguration('deadman_required', default='true')
    deadman_topic = LaunchConfiguration('deadman_topic', default='/deadman_enabled')
    palm_pose_timeout_sec = LaunchConfiguration('palm_pose_timeout_sec', default='0.3')

    # Gazebo起動
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('amir_gazebo'),
                'launch', 'amir_gazebo.launch.py'))
    )

    # コントローラーspawner（15秒後）
    joint_state_spawner = TimerAction(period=15.0, actions=[
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_controller',
                       '--controller-manager', '/controller_manager'],
            output='screen',
        )
    ])

    arm_spawner = TimerAction(period=17.0, actions=[
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['arm_controller',
                       '--controller-manager', '/controller_manager'],
            output='screen',
        )
    ])

    gripper_spawner = TimerAction(period=19.0, actions=[
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['gripper_controller',
                       '--controller-manager', '/controller_manager'],
            output='screen',
        )
    ])

    # amir_irt_mro全ノード（20秒後）
    irt_mro_launch = TimerAction(period=20.0, actions=[
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('amir_irt_mro'),
                    'launch', 'amir_irt_mro.launch.py')),
            launch_arguments={
                'use_sim': 'true',
                'mode': 'sim',
                'real_unit': 'mrad',
                'real_output': 'joint_trajectory',
                'deadman_required': deadman_required,
                'deadman_topic': deadman_topic,
                'palm_pose_timeout_sec': palm_pose_timeout_sec,
                'use_sim_time': 'true',
                'start_initial_position': 'false',
            }.items(),
        )
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'deadman_required',
            default_value='true',
            description='Require /deadman_enabled true before Gazebo commands'),
        DeclareLaunchArgument(
            'deadman_topic',
            default_value='/deadman_enabled',
            description='std_msgs/Bool deadman topic'),
        DeclareLaunchArgument(
            'palm_pose_timeout_sec',
            default_value='0.3',
            description='Stop commands when /palm_pose is stale'),
        gazebo_launch,
        joint_state_spawner,
        arm_spawner,
        gripper_spawner,
        irt_mro_launch,
    ])
