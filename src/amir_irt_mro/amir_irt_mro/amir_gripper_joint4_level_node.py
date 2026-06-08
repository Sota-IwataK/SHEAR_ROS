#!/usr/bin/env python3
import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class AmirGripperJoint4LevelNode(Node):
    def __init__(self):
        super().__init__('amir_gripper_joint4_level_node')

        self.declare_parameter(
            'joint_names',
            ['Joint_1', 'Joint_2', 'Joint_3', 'Joint_4', 'Joint_5'])
        self.declare_parameter('desired_pitch_rad', 0.0)
        self.declare_parameter('joint4_offset_rad', 0.0)
        self.declare_parameter('joint4_min_rad', -2.79)
        self.declare_parameter('joint4_max_rad', 2.79)
        self.declare_parameter('trajectory_time_sec', 1.0)
        self.declare_parameter('joint_state_topic', '/joint_states')
        self.declare_parameter(
            'arm_command_topic', '/arm_controller/joint_trajectory')
        self.declare_parameter('grasp_command_topic', '/grasp_command')
        self.declare_parameter('continuous_leveling', False)
        self.declare_parameter('level_rate_hz', 5.0)
        self.declare_parameter('joint4_deadband_rad', 0.02)
        self.declare_parameter('max_joint4_step_rad', 0.05)

        self.joint_names = list(self.get_parameter('joint_names').value)
        self.desired_pitch_rad = float(self.get_parameter('desired_pitch_rad').value)
        self.joint4_offset_rad = float(self.get_parameter('joint4_offset_rad').value)
        self.joint4_min_rad = float(self.get_parameter('joint4_min_rad').value)
        self.joint4_max_rad = float(self.get_parameter('joint4_max_rad').value)
        self.trajectory_time_sec = float(self.get_parameter('trajectory_time_sec').value)
        self.continuous_leveling = bool(
            self.get_parameter('continuous_leveling').value)
        self.level_rate_hz = float(self.get_parameter('level_rate_hz').value)
        self.joint4_deadband_rad = float(
            self.get_parameter('joint4_deadband_rad').value)
        self.max_joint4_step_rad = float(
            self.get_parameter('max_joint4_step_rad').value)
        joint_state_topic = str(self.get_parameter('joint_state_topic').value)
        arm_command_topic = str(self.get_parameter('arm_command_topic').value)
        grasp_command_topic = str(self.get_parameter('grasp_command_topic').value)

        self.latest_joint_positions = None

        self.grasp_pub = self.create_publisher(String, grasp_command_topic, 10)
        self.arm_pub = self.create_publisher(JointTrajectory, arm_command_topic, 10)
        self.create_subscription(
            String, '/amir/gripper_cmd', self.gripper_cmd_cb, 10)
        self.create_subscription(
            String, '/amir/level_joint4_once', self.level_joint4_cb, 10)
        self.create_subscription(
            JointState, joint_state_topic, self.joint_state_cb, 10)

        if self.continuous_leveling:
            if self.level_rate_hz <= 0.0:
                self.get_logger().warn(
                    'level_rate_hz must be positive. Using 5.0 Hz.')
                self.level_rate_hz = 5.0
            self.create_timer(1.0 / self.level_rate_hz, self.level_timer_cb)

        self.get_logger().info('AmirGripperJoint4LevelNode started')

    def joint_state_cb(self, msg):
        self.latest_joint_positions = {
            name: position
            for name, position in zip(msg.name, msg.position)
        }

    def gripper_cmd_cb(self, msg):
        cmd = msg.data.strip().lower()
        if cmd not in ('open', 'close'):
            self.get_logger().warn(f'Invalid gripper command: {msg.data}')
            return

        self.grasp_pub.publish(String(data=cmd))
        self.get_logger().info(f'Forwarded gripper command: {cmd}')

    def level_joint4_cb(self, msg):
        cmd = msg.data.strip().lower()
        if cmd != 'level':
            self.get_logger().warn(f'Invalid level_joint4_once command: {msg.data}')
            return

        self.publish_level_trajectory(step_limited=False, warn_on_skip=True)

    def level_timer_cb(self):
        self.publish_level_trajectory(step_limited=True, warn_on_skip=False)

    def publish_level_trajectory(self, step_limited, warn_on_skip):
        if self.latest_joint_positions is None:
            self._log_skip(
                'No joint_states received yet. JointTrajectory is not sent.',
                warn_on_skip)
            return

        missing = [
            joint_name for joint_name in self.joint_names
            if joint_name not in self.latest_joint_positions
        ]
        if missing:
            self._log_skip(
                f'Missing joints in latest joint_states: {missing}. '
                'JointTrajectory is not sent.',
                warn_on_skip)
            return

        required_level_joints = ['Joint_2', 'Joint_3', 'Joint_4']
        missing_required = [
            joint_name for joint_name in required_level_joints
            if joint_name not in self.joint_names
        ]
        if missing_required:
            self._log_skip(
                f'joint_names does not include {missing_required}. '
                'JointTrajectory is not sent.',
                warn_on_skip)
            return

        positions = [
            float(self.latest_joint_positions[joint_name])
            for joint_name in self.joint_names
        ]
        joint2 = positions[self.joint_names.index('Joint_2')]
        joint3 = positions[self.joint_names.index('Joint_3')]
        joint4_index = self.joint_names.index('Joint_4')
        current_joint4 = positions[joint4_index]

        joint4_target = (
            self.desired_pitch_rad
            - (joint2 + joint3)
            + self.joint4_offset_rad
        )
        joint4_target = max(
            self.joint4_min_rad,
            min(self.joint4_max_rad, joint4_target))
        new_joint4 = joint4_target

        if step_limited:
            joint4_error = joint4_target - current_joint4
            deadband = max(0.0, self.joint4_deadband_rad)
            if abs(joint4_error) < deadband:
                return

            max_step = max(0.0, self.max_joint4_step_rad)
            if max_step <= 0.0:
                return

            joint4_step = self._clamp(joint4_error, -max_step, max_step)
            new_joint4 = current_joint4 + joint4_step
            new_joint4 = self._clamp(
                new_joint4, self.joint4_min_rad, self.joint4_max_rad)

        positions[joint4_index] = new_joint4

        trajectory = JointTrajectory()
        trajectory.header.stamp = self.get_clock().now().to_msg()
        trajectory.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = self._duration_from_sec(self.trajectory_time_sec)
        trajectory.points = [point]

        self.arm_pub.publish(trajectory)
        self.get_logger().info(
            f'Published Joint_4 level trajectory: '
            f'Joint_4={new_joint4:.3f} rad')

    def _log_skip(self, message, warn):
        if warn:
            self.get_logger().warn(message)
        else:
            self.get_logger().debug(message)

    @staticmethod
    def _clamp(value, min_value, max_value):
        return max(min_value, min(max_value, value))

    @staticmethod
    def _duration_from_sec(seconds):
        seconds = max(0.0, float(seconds))
        sec = int(seconds)
        nanosec = int((seconds - sec) * 1_000_000_000)
        return Duration(sec=sec, nanosec=nanosec)


def main(args=None):
    rclpy.init(args=args)
    node = AmirGripperJoint4LevelNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
