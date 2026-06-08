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

    darknet_ros_dir = get_package_share_directory('darknet_ros')
    darknet_launch_dir = os.path.join(darknet_ros_dir, "launch") 

    start_observer_node = Node(
            parameters=[{"namespace": namespace}],
            package='observation',
            executable='observer',
            name='observer',
            output='screen',
            namespace=namespace,
            remappings=[('/tf', 'tf'),('/tf_static', 'tf_static')])

    start_planning_scene_node = Node(
            parameters=[{"namespace": namespace}],
            package='observation',
            executable='planning_scene',
            name='planning_scene',
            output='screen',
            namespace=namespace,
            remappings=[('/tf', 'tf'),('/tf_static', 'tf_static')])

        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     arguments = ['0', '0', '0', '0', '0', '0', 'map', 'odom']),
        
    # start_static_tf_node = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     arguments = ['0.2', '0', '0.08', '3.14159', '3.14159', '0', 'base_link', 'laser'],
    #     namespace=namespace,
    #     remappings=[('/tf_static', 'tf_static')])


    # RGB画像とDepth画像の座標系を揃えるRegistrationの下処理
    PointCloudXyzrgbNode = launch_ros.actions.ComposableNodeContainer(
            name='container',
            namespace=namespace,
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                # Driver itself
                launch_ros.descriptions.ComposableNode(
                    package='depth_image_proc',
                    namespace=namespace,
                    plugin='depth_image_proc::PointCloudXyzrgbNode',
                    name='point_cloud_xyzrgb_node',
                    remappings=[('rgb/camera_info', 'intel_realsense_r200_rgb/camera_info'),
                                ('rgb/image_rect_color', 'intel_realsense_r200_rgb/image_raw'),
                                ('depth_registered/image_rect',
                                 'intel_realsense_r200_depth/image_raw'),
                                ('points', 'camera/depth_registered/points')]
                ),
            ],
            output='screen',
        )

    start_object_callback_node = Node(
            # parameters=[{"robot_namespace": robot_namespace}],
            package='observation',
            executable='object_callback',
            name='object_callback',
            output='screen',
            namespace=namespace,
            parameters=
                [{"camera_pointcloud_topic": "camera/depth_registered/points",
                "camera_rgb_topic":"intel_realsense_r200_rgb/image_raw",
                "bounding_boxes_topic":"darknet_ros/bounding_boxes",
                "show_image_window_object_callback":True}],
            remappings=[('/tf', 'tf'),('/tf_static', 'tf_static')])

    darknet_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(darknet_launch_dir, "darknet_ros_yogo_realsense.launch.py")),
        # PythonLaunchDescriptionSource(os.path.join(darknet_launch_dir, "darknet_ros.launch.py")),
        launch_arguments={
            # "namespace": namespace,
            # "use_namespace": "True",
            # "rviz_config": rviz_config_file,
        }.items(),
    )



    ld = LaunchDescription()

    ld.add_action(declare_namespace_cmd)
    ld.add_action(start_observer_node)
    # ld.add_action(start_planning_scene_node)
    # ld.add_action(PointCloudXyzrgbNode)
    # ld.add_action(start_object_callback_node)
    # ld.add_action(darknet_node)
    
    

    return ld

