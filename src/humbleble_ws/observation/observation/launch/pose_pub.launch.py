import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import launch_ros.actions
import launch_ros.descriptions
# from launch_ros.actions import IncludeLaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    # Create the launch configuration variables
    namespace = LaunchConfiguration("namespace")

    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace", default_value="robot1", description="Top-level namespace"
    )


    start_pose_pub_node = Node(
            parameters=[{"namespace": namespace}],
            package='observation',
            executable='my_pose_publisher',
            name='my_pose_publisher',
            output='screen',
            namespace=namespace,
            remappings=[('/tf', 'tf'),('/tf_static', 'tf_static')])



    ld = LaunchDescription()
    ld.add_action(declare_namespace_cmd)
    ld.add_action(start_pose_pub_node)

    
    return ld

