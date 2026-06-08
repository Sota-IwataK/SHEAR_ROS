from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    # Create the launch configuration variables
    robot_namespace = LaunchConfiguration("robot_namespace")

    declare_namespace_cmd = DeclareLaunchArgument(
        "robot_namespace", default_value="mecanum2", description="Top-level namespace"
    )

    start_send_goal_node = Node(
            parameters=[{"robot_namespace": robot_namespace, 'target_x': 0.0, 'target_y': 0.0}],
            package='observation',
            executable='nav2_send_goal',
            name='nav2_send_goal',
            output='screen',
            namespace=robot_namespace,
            remappings=[('/tf', 'tf')])


    ld = LaunchDescription()

    ld.add_action(declare_namespace_cmd)
    ld.add_action(start_send_goal_node)

    return ld

