import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_launch_dir = os.path.join(bringup_dir, 'launch')

    this_dir = get_package_share_directory('mecanum_navigation2')
    this_launch_dir = os.path.join(this_dir, 'launch')

    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config_file = LaunchConfiguration('rviz_config_file')

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1'
    )

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='',
        description='Top-level namespace'
    )
    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace', default_value='false',
        description='Whether to apply a namespace to the navigation stack'
    )
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(this_dir, 'params', 'amir.yaml'),
        description='Full path to the ROS2 parameters file'
    )
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack'
    )
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Whether to start RViz'
    )
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(
            get_package_share_directory('nav2_bringup'),
            'rviz', 'nav2_default_view.rviz'),
        description='Full path to the RViz config file'
    )

    # slam_toolbox: online async mapping (builds map while driving)
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(this_launch_dir, 'slam_toolbox_launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    # nav2: planner + controller + behavior (no localization — slam_toolbox handles TF)
    bringup_cmd_group = GroupAction([
        PushRosNamespace(condition=IfCondition(use_namespace), namespace=namespace),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_launch_dir, 'navigation_launch.py')
            ),
            launch_arguments={
                'namespace': namespace,
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'params_file': params_file,
                'use_lifecycle_mgr': 'false',
                'map_subscribe_transient_local': 'true',
            }.items(),
        ),
    ])

    # Relay Nav2 /cmd_vel output to /rover_twist (→ mecanum drive controller)
    cmd_vel_relay = Node(
        package='mecanum_navigation2',
        executable='cmd_vel_relay.py',
        name='cmd_vel_relay',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_launch_dir, 'rviz_launch.py')
        ),
        condition=IfCondition(use_rviz),
        launch_arguments={
            'namespace': namespace,
            'use_namespace': 'false',
            'rviz_config': rviz_config_file,
        }.items(),
    )

    ld = LaunchDescription()
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_rviz_config_file_cmd)

    ld.add_action(slam_toolbox_launch)
    ld.add_action(bringup_cmd_group)
    ld.add_action(cmd_vel_relay)
    ld.add_action(rviz_cmd)

    return ld
