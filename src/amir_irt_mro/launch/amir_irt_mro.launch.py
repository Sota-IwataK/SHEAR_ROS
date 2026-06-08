#!/usr/bin/env python3
"""
amir_irt_mro 統合launchファイル
全ノードを一括起動する
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # 引数
    use_sim = LaunchConfiguration('use_sim', default='true')
    mode = LaunchConfiguration('mode', default='sim')
    real_unit = LaunchConfiguration('real_unit', default='mrad')
    real_output = LaunchConfiguration('real_output', default='joint_trajectory')
    deadman_required = LaunchConfiguration('deadman_required', default='true')
    deadman_topic = LaunchConfiguration('deadman_topic', default='/deadman_enabled')
    palm_pose_timeout_sec = LaunchConfiguration('palm_pose_timeout_sec', default='0.3')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    start_initial_position = LaunchConfiguration('start_initial_position', default='false')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim',
            default_value='true',
            description='Gazebo simulation mode (true) or real robot (false)'),
        DeclareLaunchArgument(
            'mode',
            default_value='sim',
            description='Control mode: sim, real, or dry_run'),
        DeclareLaunchArgument(
            'real_unit',
            default_value='mrad',
            description='Real AmirCmd unit: mrad or rad'),
        DeclareLaunchArgument(
            'real_output',
            default_value='joint_trajectory',
            description='Real output path: joint_trajectory or amir_cmd'),
        DeclareLaunchArgument(
            'deadman_required',
            default_value='true',
            description='Require /deadman_enabled true before sending commands'),
        DeclareLaunchArgument(
            'deadman_topic',
            default_value='/deadman_enabled',
            description='std_msgs/Bool deadman topic'),
        DeclareLaunchArgument(
            'palm_pose_timeout_sec',
            default_value='0.3',
            description='Stop command output when /palm_pose is stale'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock'),
        DeclareLaunchArgument(
            'start_initial_position',
            default_value='false',
            description='Start initial-position node. Default false for safety.'),

        # 1. 初期姿勢設定（最初に起動）
        Node(
            package='amir_irt_mro',
            executable='amir_initial_position',
            name='amir_initial_position',
            output='screen',
            condition=IfCondition(start_initial_position),
            parameters=[{
                'use_sim_time': use_sim_time,
                'auto_send': False,
                'deadman_required': deadman_required,
                'deadman_topic': deadman_topic,
            }],
        ),

        # 2. アーム軌道制御（2秒後）
        TimerAction(period=2.0, actions=[
            Node(
                package='amir_irt_mro',
                executable='amir_trajectory_node',
                name='amir_trajectory_node',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'use_sim': use_sim,
                    'mode': mode,
                    'real_unit': real_unit,
                    'real_output': real_output,
                    'deadman_required': deadman_required,
                    'deadman_topic': deadman_topic,
                    'palm_pose_timeout_sec': palm_pose_timeout_sec,
                }],
            ),
        ]),

        # 3. グリッパー制御（2秒後）
        TimerAction(period=2.0, actions=[
            Node(
                package='amir_irt_mro',
                executable='amir_gripper',
                name='amir_gripper',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}],
            ),
        ]),

        # 4. ベース移動（2秒後）
        TimerAction(period=2.0, actions=[
            Node(
                package='amir_irt_mro',
                executable='amir_base_move',
                name='amir_base_move',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}],
            ),
        ]),

        # 5. 経路計画（3秒後）
        TimerAction(period=3.0, actions=[
            Node(
                package='amir_irt_mro',
                executable='amir_path_planner',
                name='amir_path_planner',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}],
            ),
        ]),

        # 6. 座標変換（3秒後）
        TimerAction(period=3.0, actions=[
            Node(
                package='amir_irt_mro',
                executable='amir_affine_transform',
                name='amir_affine_transform',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}],
            ),
        ]),

        # 7. 競合検出（4秒後）
        TimerAction(period=4.0, actions=[
            Node(
                package='amir_irt_mro',
                executable='conflict_detector',
                name='conflict_detector',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}],
            ),
        ]),

        # 8. 情報フィルタリング（4秒後）
        TimerAction(period=4.0, actions=[
            Node(
                package='amir_irt_mro',
                executable='info_filter',
                name='info_filter',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}],
            ),
        ]),
    ])
