import math
import os
import tempfile

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _gen_sdf_and_spawn(context, *args, **kwargs):
    entity = LaunchConfiguration('entity').perform(context)
    shape  = LaunchConfiguration('shape').perform(context)

    x     = float(LaunchConfiguration('x').perform(context))
    y     = float(LaunchConfiguration('y').perform(context))
    z     = float(LaunchConfiguration('z').perform(context))
    roll  = float(LaunchConfiguration('roll').perform(context))
    pitch = float(LaunchConfiguration('pitch').perform(context))
    yaw   = float(LaunchConfiguration('yaw').perform(context))
    mass  = float(LaunchConfiguration('mass').perform(context))

    # Ignition Gazebo は Classic Gazebo より摩擦計算が正確なため
    # kp/kd は Classic 同等値で十分 (DART/ODE どちらでも機能する)
    kp = 1e6
    kd = 1e3
    mu = 3.0

    surface_xml = f"""
        <surface>
          <friction>
            <ode>
              <mu>{mu}</mu>
              <mu2>{mu}</mu2>
            </ode>
          </friction>
          <contact>
            <ode>
              <kp>{kp}</kp>
              <kd>{kd}</kd>
              <max_vel>0.01</max_vel>
              <min_depth>0.001</min_depth>
            </ode>
          </contact>
        </surface>"""

    if shape == 'cylinder':
        radius = float(LaunchConfiguration('radius').perform(context))
        height = float(LaunchConfiguration('size_z').perform(context))
        ixx = iyy = (1.0 / 12.0) * mass * (3.0 * radius**2 + height**2)
        izz = 0.5 * mass * radius**2
        geom_xml = f"""<cylinder>
              <radius>{radius}</radius>
              <length>{height}</length>
            </cylinder>"""
        link_name = 'cylinder_link'
    else:
        size_x = float(LaunchConfiguration('size_x').perform(context))
        size_y = float(LaunchConfiguration('size_y').perform(context))
        size_z = float(LaunchConfiguration('size_z').perform(context))
        ixx = (1.0 / 12.0) * mass * (size_y**2 + size_z**2)
        iyy = (1.0 / 12.0) * mass * (size_x**2 + size_z**2)
        izz = (1.0 / 12.0) * mass * (size_x**2 + size_y**2)
        geom_xml = f"""<box>
              <size>{size_x} {size_y} {size_z}</size>
            </box>"""
        link_name = 'box_link'

    sdf = f'''<?xml version="1.0" ?>
<sdf version="1.8">
  <model name="{entity}">
    <static>false</static>
    <link name="{link_name}">
      <inertial>
        <mass>{mass}</mass>
        <inertia>
          <ixx>{ixx}</ixx>
          <iyy>{iyy}</iyy>
          <izz>{izz}</izz>
          <ixy>0.0</ixy>
          <ixz>0.0</ixz>
          <iyz>0.0</iyz>
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry>
          {geom_xml}
        </geometry>
        {surface_xml}
      </collision>
      <visual name="visual">
        <geometry>
          {geom_xml}
        </geometry>
        <material>
          <ambient>0.9 0.5 0.1 1.0</ambient>
          <diffuse>0.9 0.5 0.1 1.0</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
'''

    tmpdir = tempfile.mkdtemp(prefix='spawn_obj_')
    sdf_path = os.path.join(tmpdir, f'{entity}.sdf')
    with open(sdf_path, 'w') as f:
        f.write(sdf)

    # Ignition Gazebo: ign service で既存モデルを削除 (world 名は amir_world.sdf の name="default")
    delete_cmd = ExecuteProcess(
        cmd=[
            'ign', 'service',
            '-s', '/world/default/remove',
            '--reqtype', 'ignition.msgs.Entity',
            '--reptype', 'ignition.msgs.Boolean',
            '--timeout', '1000',
            '--req', f'name: "{entity}" type: MODEL',
        ],
        output='log',
    )

    # Ignition Gazebo: ros_gz_sim create でスポーン
    spawn_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', sdf_path,
            '-name', entity,
            '-x', str(x), '-y', str(y), '-z', str(z),
            '-R', str(roll), '-P', str(pitch), '-Y', str(yaw),
            '-allow_renaming', 'false',
        ],
        output='screen',
    )

    return [
        delete_cmd,
        TimerAction(period=0.5, actions=[spawn_node]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('entity', default_value='target_obj'),
        DeclareLaunchArgument('shape',  default_value='box', description="'cylinder' or 'box'"),
        DeclareLaunchArgument('x',     default_value='0.4'),
        DeclareLaunchArgument('y',     default_value='0.0'),
        DeclareLaunchArgument('z',     default_value='0.05'),
        DeclareLaunchArgument('roll',  default_value='0.0'),
        DeclareLaunchArgument('pitch', default_value='0.0'),
        DeclareLaunchArgument('yaw',   default_value='0.0'),
        DeclareLaunchArgument('radius',  default_value='0.025'),
        DeclareLaunchArgument('size_z',  default_value='0.10'),
        DeclareLaunchArgument('size_x',  default_value='0.05'),
        DeclareLaunchArgument('size_y',  default_value='0.05'),
        DeclareLaunchArgument('mass',    default_value='0.1'),

        OpaqueFunction(function=_gen_sdf_and_spawn),
    ])
