import os
# from rclpy.parameter import Parameter
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time")
    namespace = LaunchConfiguration("namespace")

    declare_use_sim_time_argument = DeclareLaunchArgument(
        "use_sim_time", default_value="true", description="Use simulation/Gazebo clock"
    )

    declare_namespace_argument = DeclareLaunchArgument(
        "namespace",
        default_value="mecanum1",
        description="Namespace for the explore node",
    )


    
    # global_parameters = os.path.join(
    #     get_package_share_directory('observation'),
    #     'config',
    #     'global_params.yaml'
    # )
    global_param_node = Node(
        package='observation',
        executable='robot_manager',
        name='robot_manager',
        namespace=namespace,
        parameters=[{"use_sim_time": use_sim_time},
                    {'robot_name': namespace}],
        output='screen',
    
        # emulate_tty=True
        # prefix=['stdbuf -o L']  #これがないと表示されない。foxy以降はemulate_tty=True

    )

    pose_node = Node(
        package='observation',
        executable='my_pose_publisher',
        name='my_pose_publisher',
        namespace=namespace,
        parameters=[{"use_sim_time": use_sim_time},
                    {'robot_name': namespace}],
        output='screen',
        remappings = [
        ("/tf", "tf"),
        ("/tf_static", "tf_static")]
    
        # emulate_tty=True
        # prefix=['stdbuf -o L']  #これがないと表示されない。foxy以降はemulate_tty=True
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_namespace_argument)
    ld.add_action(global_param_node)
    ld.add_action(pose_node)
    return ld