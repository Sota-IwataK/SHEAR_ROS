import os
# from rclpy.parameter import Parameter
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    ld = LaunchDescription()

    pick_node = Node(
        package='amir_operation',
        executable='pick',
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

    obj_pub_node = Node(
    package='amir_operation',
    executable='object_pub',
    output='screen',
    )

    before_arm_node = Node(
        package='amir_operation',
        executable='before_arm',
        output='screen',
    )
    after_arm_node = Node(
        package='amir_operation',
        executable='after_arm',
        output='screen',
    )
    search_amir_node = Node(
        package='amir_operation',
        executable='search_amir',
        output='screen',
    )
    return LaunchDescription([
        pick_node,
        place_node,
        move_arm_node,
        grasp_node,
        move_meca_node,
        before_arm_node,
        after_arm_node,
        search_amir_node
        # obj_pub_node
    ])

    # ld.add_action(pick_node)
    # ld.add_action(place_node)
    # ld.add_action(move_arm_node)
    # ld.add_action(grasp_node)
    # return ld