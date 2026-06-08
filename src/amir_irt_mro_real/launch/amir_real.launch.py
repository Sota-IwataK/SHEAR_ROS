#!/usr/bin/env python3
"""Real Amir launch wrapper with explicit safety gates.

This file intentionally does not start a non-existent amir_driver_node.
amir_driver is a ros2_control hardware plugin; start_ros2_control must be
enabled explicitly after dry-run checks.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    real_unit = LaunchConfiguration('real_unit', default='mrad')
    real_output = LaunchConfiguration('real_output', default='joint_trajectory')
    deadman_required = LaunchConfiguration('deadman_required', default='true')
    deadman_topic = LaunchConfiguration('deadman_topic', default='/deadman_enabled')
    palm_pose_timeout_sec = LaunchConfiguration('palm_pose_timeout_sec', default='0.3')
    start_ros2_control = LaunchConfiguration('start_ros2_control', default='false')

    # 変更根拠: amir_driver_node は CMake/setup.py/install に存在しない。
    # 期待される効果: 実機 launch の失敗原因を取り除き、ros2_control 起動を明示操作にする。
    ros2_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('amir_bringup'),
                'launch', 'super_amir_moveit.launch.py')),
        condition=IfCondition(start_ros2_control),
    )

    irt_mro_launch = TimerAction(period=2.0, actions=[
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('amir_irt_mro'),
                    'launch', 'amir_irt_mro.launch.py')),
            launch_arguments={
                'use_sim': 'false',
                'mode': 'real',
                'real_unit': real_unit,
                'real_output': real_output,
                'deadman_required': deadman_required,
                'deadman_topic': deadman_topic,
                'palm_pose_timeout_sec': palm_pose_timeout_sec,
                'use_sim_time': 'false',
                'start_initial_position': 'false',
            }.items(),
        )
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'real_unit',
            default_value='mrad',
            description='Real output unit preview/direct AmirCmd unit: mrad or rad'),
        DeclareLaunchArgument(
            'real_output',
            default_value='joint_trajectory',
            description='Real output path: joint_trajectory or amir_cmd'),
        DeclareLaunchArgument(
            'deadman_required',
            default_value='true',
            description='Require /deadman_enabled true before real commands'),
        DeclareLaunchArgument(
            'deadman_topic',
            default_value='/deadman_enabled',
            description='std_msgs/Bool deadman topic'),
        DeclareLaunchArgument(
            'palm_pose_timeout_sec',
            default_value='0.3',
            description='Stop commands when /palm_pose is stale'),
        DeclareLaunchArgument(
            'start_ros2_control',
            default_value='false',
            description='Explicitly start existing ros2_control real hardware launch'),
        LogInfo(msg=(
            'amir_driver_node is not started because it is not defined. '
            'Use start_ros2_control:=true only after dry-run checks.'
        )),
        ros2_control_launch,
        irt_mro_launch,
    ])
