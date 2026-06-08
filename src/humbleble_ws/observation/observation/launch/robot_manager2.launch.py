import os
# from rclpy.parameter import Parameter
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
def generate_launch_description():
    ld = LaunchDescription()
    # global_parameters = os.path.join(
    #     get_package_share_directory('observation'),
    #     'config',
    #     'global_params.yaml'
    # )
    global_param_node = Node(
        package='observation',
        executable='robot_manager',
        name='robot_manager',
        namespace="mecanum2",
        parameters=[{'robot_name': 'mecanum2'}],
        output='screen',
        # prefix=['stdbuf -o L']  #これがないと表示されない。foxy以降はemulate_tty=True

    )
    ld.add_action(global_param_node)
    return ld