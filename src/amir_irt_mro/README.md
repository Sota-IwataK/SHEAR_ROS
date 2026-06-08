# amir_irt_mro operation memo

This package receives MR/Unity PalmPose on `/palm_pose` and computes Amir arm
joint targets. The control node is `amir_trajectory_node`.

## Unit policy

- internal unit: rad
- JOINT_LIMITS: rad
- sim output: `trajectory_msgs/JointTrajectory` in rad
- real output through `joint_trajectory`: rad; `amir_driver` converts to mrad
- real output through `amir_cmd`: mrad if `real_unit:=mrad`, rad if `real_unit:=rad`

`real_unit:=mrad` defaults to the safer assumption from `AmirCmd.msg` comments
and `amir_driver/src/amir_hardware_interfaces.cpp`, which converts rad to mrad
before publishing `AmirCmd`.

## Modes

- `mode:=sim`: publish JointTrajectory to `/arm_controller/joint_trajectory`
- `mode:=real`: publish using `real_output`
- `mode:=dry_run`: publish no robot command; log PalmPose, IK result, planned values, Deadman, and Hz

`real_output` values:

- `joint_trajectory`: use ros2_control arm controller
- `amir_cmd`: publish `amir_interfaces/AmirCmd` to `/motor_sub`

Direct `amir_cmd` output is not the default because the real low-level transport
and the sixth `AmirCmd` axis must still be confirmed.

## Deadman

The control node subscribes to `/deadman_enabled` as `std_msgs/Bool`.

With `deadman_required:=true`, commands are published only when:

- `/deadman_enabled` is `true`
- `/palm_pose` is fresh
- the node has initialized from `/joint_states`, except in `dry_run`

When Deadman is OFF or PalmPose times out, the node sends no new robot command.
It does not send a zero pose because that can move the real robot unexpectedly.

Default timeout:

- `palm_pose_timeout_sec:=0.3`

## Topics

- `/palm_pose`: `geometry_msgs/PoseStamped`, Unity/Quest PalmPose input
- `/deadman_enabled`: `std_msgs/Bool`, command enable
- `/joint_states`: `sensor_msgs/JointState`, current joint state
- `/arm_controller/joint_trajectory`: `trajectory_msgs/JointTrajectory`, sim or ros2_control real path
- `/motor_sub`: `amir_interfaces/AmirCmd`, direct real command path
- `/amir_metrics`: `std_msgs/Float32MultiArray`, existing metrics output
- `/phase_name`: `std_msgs/String`, FSM phase
- `/identified_bottle_pose`: `std_msgs/Float32MultiArray`, optional pregrasp input

## Parameters

- `mode`: `sim`, `real`, or `dry_run`
- `real_unit`: `mrad` or `rad`
- `real_output`: `joint_trajectory` or `amir_cmd`
- `arm_trajectory_topic`: default `/arm_controller/joint_trajectory`
- `real_cmd_topic`: default `/motor_sub`
- `deadman_required`: default `true`
- `deadman_topic`: default `/deadman_enabled`
- `palm_pose_timeout_sec`: default `0.3`
- `summary_log_period_sec`: default `1.0`
- `L1`, `L2`, `L3`, `alpha`, `k_null`, `joint_vel`: IK/control tuning

## Gazebo check

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch amir_irt_mro_sim amir_sim.launch.py
```

In another terminal:

```bash
source ~/ros2_ws/install/setup.bash
ros2 topic pub /deadman_enabled std_msgs/msg/Bool "{data: true}" -r 5
ros2 topic hz /palm_pose
ros2 topic hz /arm_controller/joint_trajectory
ros2 topic echo /amir_metrics
```

## Palm IK node tuning examples

`amir_palm_ik_node` keeps `mode:=sim` as the default. In `mode:=sim` it does
not publish `/arm_controller/joint_trajectory`; use `mode:=gazebo` for Gazebo
trajectory-controller checks.

Gazebo tuning example:

```bash
ros2 run amir_irt_mro amir_palm_ik_node --ros-args \
  -p mode:=gazebo \
  -p palm_motion_gain:=0.12 \
  -p max_target_step_m:=0.003 \
  -p trajectory_time_from_start_sec:=0.35
```

First real-hardware check example:

```bash
ros2 run amir_irt_mro amir_palm_ik_node --ros-args \
  -p mode:=real \
  -p palm_motion_gain:=0.05 \
  -p max_target_step_m:=0.001 \
  -p trajectory_time_from_start_sec:=0.8
```

The default palm-axis conversion is:

- `robot_x = +palm_z`
- `robot_y = +palm_x`
- `robot_z = +palm_y`

## Dry-run check

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch amir_irt_mro amir_irt_mro.launch.py \
  mode:=dry_run use_sim:=false use_sim_time:=false real_unit:=mrad
```

In another terminal, publish Deadman and a test PalmPose:

```bash
source ~/ros2_ws/install/setup.bash
ros2 topic pub /deadman_enabled std_msgs/msg/Bool "{data: true}" -r 5
ros2 topic pub /palm_pose geometry_msgs/msg/PoseStamped \
"{header: {frame_id: base_link}, pose: {position: {x: 0.25, y: 0.0, z: 0.75}, orientation: {w: 1.0}}}" -r 20
```

Confirm the 1Hz summary log shows `cmd_hz=0.00` in `dry_run`.

## Real pre-checklist

- `/palm_pose` arrives at the expected Hz.
- Deadman OFF produces no command output.
- `dry_run` IK values are finite and within expected range.
- JOINT_LIMITS are rad internal values.
- `real_unit:=mrad` previews x1000 conversion before real output.
- Gazebo works with `mode:=sim`.
- The real driver executable name matches launch. Current finding: `amir_driver_node` does not exist.
- `/dev/ttyUSB0` and baudrate `3000000` are correct for the lower Amir transport.
- Joint_2 offset is confirmed on the real robot.

## Real launch

Safe wrapper:

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch amir_irt_mro_real amir_real.launch.py
```

This starts `amir_trajectory_node` in `mode:=real` but does not start
`amir_driver_node`, because that executable is not defined.

To explicitly try the existing ros2_control real hardware path after dry-run:

```bash
ros2 launch amir_irt_mro_real amir_real.launch.py start_ros2_control:=true real_output:=joint_trajectory
```

For direct AmirCmd output, only after confirming the lower-level `/motor_sub`
consumer and the sixth actuator convention:

```bash
ros2 launch amir_irt_mro_real amir_real.launch.py real_output:=amir_cmd real_unit:=mrad
```

## Known unconfirmed points

- Whether the real lower-level Amir driver consumes `/motor_sub` directly in this workspace.
- Whether direct `AmirCmd` should command the sixth axis or hold it.
- Whether all real Amir command offsets match `AMIR_DRIVER_INITIAL_POSITION_MRAD`.
- Joint_2 real offset.
- Gazebo controller-side unit handling, although ros2_control position interfaces are normally rad.
- `amir_driver_node` is referenced by the old real launch but is not built or installed.
