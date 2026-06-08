import os
# from rclpy.parameter import Parameter
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    ld = LaunchDescription()

    # pick_node = Node(
    #     package='amir_operation',
    #     executable='pick',
    #     output='screen',
    # )

    pick_node = Node(
        package='amir_operation',
        executable='pick_using_d_optimality',
        output='screen',
    )

    place_node = Node(
        package='amir_operation',
        executable='place',
        output='screen',
    )

    move_arm_node = Node(
        package='amir_operation',
        executable='move_arm',
        output='screen',
    )

    grasp_node = Node(
        package='amir_operation',
        executable='grasp',
        output='screen',
    )

    move_meca_node = Node(
        package='amir_operation',
        executable='move_meca',
        output='screen',
    )

    return LaunchDescription([
        pick_node,
        place_node,
        move_arm_node,
        grasp_node,
        move_meca_node
    ])

    # ld.add_action(pick_node)
    # ld.add_action(place_node)
    # ld.add_action(move_arm_node)
    # ld.add_action(grasp_node)
    # return ld