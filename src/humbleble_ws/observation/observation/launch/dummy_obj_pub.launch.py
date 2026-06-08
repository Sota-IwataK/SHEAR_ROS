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
        "robot_namespace", default_value="robot1", description="Top-level namespace"
    )

    start_observer_node = Node(
            parameters=[{"robot_namespace": robot_namespace}],
            package='observation',
            executable='dummy_obj_pub',
            name='dummy_obj_pub',
            output='screen',
            namespace=robot_namespace,
            remappings=[('/tf', 'tf'),('/tf_static', 'tf_static')])

        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     arguments = ['0', '0', '0', '0', '0', '0', 'map', 'odom']),
        
    # start_static_tf_node = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     arguments = ['0.2', '0', '0.08', '3.14159', '3.14159', '0', 'base_link', 'laser'],
    #     namespace=robot_namespace,
    #     remappings=[('/tf_static', 'tf_static')])



    ld = LaunchDescription()

    ld.add_action(declare_namespace_cmd)
    ld.add_action(start_observer_node)

    return ld

