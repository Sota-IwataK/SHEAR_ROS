import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _safe_kill_ignition(context, *args, **kwargs):
    os.system("pkill -9 -f 'ign gazebo' 2>/dev/null")
    os.system("pkill -9 -f 'ruby.*ign' 2>/dev/null")
    os.system("rm -rf /dev/shm/fastrtps_port* 2>/dev/null")
    return [LogInfo(msg="[amir_gazebo] Safe kill done")]


def generate_launch_description():
    gui = LaunchConfiguration("gui")

    amir_description_dir = get_package_share_directory("amir_description")
    amir_gazebo_dir = get_package_share_directory("amir_gazebo")
    world_file = os.path.join(amir_gazebo_dir, "worlds", "amir_world.sdf")

    arm_controllers_yaml = os.path.join(amir_gazebo_dir, "config", "arm_controllers.yaml")

    # xacro → URDF は Command 置換で実行時に処理
    # RSP がこれを robot_description パラメータとして保持・publish する
    # ros_gz_sim create は -topic /robot_description でそれを受け取りスポーン
    xacro_file = os.path.join(amir_description_dir, "urdf", "amir_mecanum3_sim.xacro")
    robot_description_content = ParameterValue(
        Command([
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            xacro_file,
        ]),
        value_type=str,
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_description_content},
            {"use_sim_time": True},
        ],
    )

    # Ignition Gazebo 起動 (ros_gz_sim の gz_sim.launch.py 経由)
    # -r: シミュレーション開始 (これがないとコントローラが起動しない)
    # IGN_GAZEBO_SYSTEM_PLUGIN_PATH は gz_sim.launch.py が LD_LIBRARY_PATH を含めて設定
    gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])
        ),
        launch_arguments=[
            ("gz_args", [" -r -v 1 ", world_file]),
            ("gz_version", "6"),
            ("on_exit_shutdown", "true"),
        ],
    )

    # ロボットを /robot_description トピックからスポーン
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic", "/robot_description",
            "-name", "amir_mecanum3",
            "-z", "0.03",
            "-allow_renaming", "false",
        ],
    )

    # /clock ブリッジ (Ignition → ROS2)
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )

    # LRF スキャンブリッジ (Ignition → ROS2)
    # front: /scan, back: /back_scan
    scan_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            "/back_scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
        ],
        output="screen",
    )

    # /rover_twist → /mecanum_drive_controller/cmd_vel 中継ノード
    # gz_ros2_control の <ros><remapping> は controller_manager に届かないため
    # 専用リレーノードで後方互換性を維持する
    rover_twist_relay = Node(
        package="mecanumrover_description",
        executable="rover_twist_relay_ign.py",
        name="rover_twist_relay_ign",
        output="log",
        parameters=[{"use_sim_time": True}],
    )

    # odom TF リレー: ros2_controllers (Humble) は odom→base_footprint TF を
    # /mecanum_drive_controller/tf_odometry に出力するため /tf に転送する
    odom_tf_relay = Node(
        package="amir_gazebo",
        executable="odom_tf_relay.py",
        name="odom_tf_relay",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    # ── コントローラ起動 (OnProcessExit で順番に起動) ──────────────────

    jsb = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager-timeout", "60",
            "--service-call-timeout", "60.0",
        ],
        output="screen",
    )

    # arm と mecanum は jsb 完了後に並行起動
    arm = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_controller",
            "-t", "joint_trajectory_controller/JointTrajectoryController",
            "-p", arm_controllers_yaml,
            "--controller-manager-timeout", "30",
            "--service-call-timeout", "30.0",
        ],
        output="screen",
    )

    mecanum = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "mecanum_drive_controller",
            "--controller-manager-timeout", "30",
            "--service-call-timeout", "30.0",
        ],
        output="screen",
    )

    gripper = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "gripper_controller",
            "-t", "position_controllers/GripperActionController",
            "-p", arm_controllers_yaml,
            "--controller-manager-timeout", "30",
            "--service-call-timeout", "30.0",
        ],
        output="screen",
    )

    # スポーン完了後 → jsb 起動
    jsb_after_spawn = RegisterEventHandler(
        OnProcessExit(target_action=spawn_robot, on_exit=[jsb])
    )
    # jsb 完了後 → arm と mecanum を並行起動
    arm_after_jsb = RegisterEventHandler(
        OnProcessExit(target_action=jsb, on_exit=[arm, mecanum])
    )
    # arm 完了後 → gripper 起動
    gripper_after_arm = RegisterEventHandler(
        OnProcessExit(target_action=arm, on_exit=[gripper])
    )

    on_shutdown_kill = RegisterEventHandler(
        OnShutdown(on_shutdown=[OpaqueFunction(function=_safe_kill_ignition)])
    )

    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true"),
        SetParameter(name="use_sim_time", value=True),
        robot_state_publisher,
        gz_launch,
        spawn_robot,
        clock_bridge,
        scan_bridge,
        rover_twist_relay,
        odom_tf_relay,
        jsb_after_spawn,
        arm_after_jsb,
        gripper_after_arm,
        on_shutdown_kill,
    ])
