#!/usr/bin/env python3
"""Right-hand mecanum rover controller for AMIR."""

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Float32MultiArray


class AmirRightHandMecanumNode(Node):
    def __init__(self):
        super().__init__('amir_right_hand_mecanum_node')

        self.declare_parameter('input_topic', '/amir/right_hand_mecanum_input')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('control_rate_hz', 30.0)
        self.declare_parameter('timeout_sec', 0.3)
        self.declare_parameter('gain_x', 0.4)
        self.declare_parameter('gain_y', 0.4)
        self.declare_parameter('gain_yaw', 0.5)
        self.declare_parameter('deadzone_pos_m', 0.02)
        self.declare_parameter('deadzone_roll_rad', 0.0872665)
        self.declare_parameter('max_linear_x', 0.03)
        self.declare_parameter('max_linear_y', 0.03)
        self.declare_parameter('max_angular_z', 0.15)
        self.declare_parameter('invert_x', False)
        self.declare_parameter('invert_y', False)
        self.declare_parameter('invert_yaw', False)

        self.input_topic = self.get_parameter('input_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.control_rate_hz = float(self.get_parameter('control_rate_hz').value)
        if self.control_rate_hz <= 0.0:
            self.get_logger().warn('control_rate_hz must be > 0. Using 30.0 Hz.')
            self.control_rate_hz = 30.0

        self.timeout_sec = float(self.get_parameter('timeout_sec').value)
        self.gain_x = float(self.get_parameter('gain_x').value)
        self.gain_y = float(self.get_parameter('gain_y').value)
        self.gain_yaw = float(self.get_parameter('gain_yaw').value)
        self.deadzone_pos_m = abs(float(self.get_parameter('deadzone_pos_m').value))
        self.deadzone_roll_rad = abs(float(self.get_parameter('deadzone_roll_rad').value))
        self.max_linear_x = abs(float(self.get_parameter('max_linear_x').value))
        self.max_linear_y = abs(float(self.get_parameter('max_linear_y').value))
        self.max_angular_z = abs(float(self.get_parameter('max_angular_z').value))
        self.invert_x = bool(self.get_parameter('invert_x').value)
        self.invert_y = bool(self.get_parameter('invert_y').value)
        self.invert_yaw = bool(self.get_parameter('invert_yaw').value)

        self.active = False
        self.input_valid = False
        self.delta_x = 0.0
        self.delta_z = 0.0
        self.roll_delta = 0.0
        self.last_msg_time = None
        self.last_log_time = self.get_clock().now()
        self.last_cmd = Twist()

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.input_sub = self.create_subscription(
            Float32MultiArray,
            self.input_topic,
            self.input_callback,
            10,
        )
        self.timer = self.create_timer(1.0 / self.control_rate_hz, self.control_loop)

        self.get_logger().info(
            f'AmirRightHandMecanumNode started. '
            f'Listening on {self.input_topic}, publishing {self.cmd_vel_topic}'
        )

    def input_callback(self, msg):
        self.last_msg_time = self.get_clock().now()

        if len(msg.data) < 5:
            self.active = False
            self.input_valid = False
            self.delta_x = 0.0
            self.delta_z = 0.0
            self.roll_delta = 0.0
            self.get_logger().warn('Input data length is less than 5. Publishing zero cmd_vel.')
            return

        values = [float(value) for value in msg.data[:5]]
        if not all(math.isfinite(value) for value in values):
            self.active = False
            self.input_valid = False
            self.delta_x = 0.0
            self.delta_z = 0.0
            self.roll_delta = 0.0
            self.get_logger().warn('Input contains NaN or inf. Publishing zero cmd_vel.')
            return

        active_flag, self.delta_x, self.delta_z, self.roll_delta, _timestamp = values
        self.active = active_flag >= 0.5
        self.input_valid = True

    def control_loop(self):
        timeout = self.is_timeout()
        if not self.input_valid or not self.active or timeout:
            cmd = Twist()
        else:
            cmd = self.make_cmd_vel()

        self.last_cmd = cmd
        self.cmd_pub.publish(cmd)
        self.log_status(timeout)

    def make_cmd_vel(self):
        delta_z = self.apply_deadzone(self.delta_z, self.deadzone_pos_m)
        delta_x = self.apply_deadzone(self.delta_x, self.deadzone_pos_m)
        roll_delta = self.apply_deadzone(self.roll_delta, self.deadzone_roll_rad)

        linear_x = self.gain_x * delta_z
        linear_y = self.gain_y * delta_x
        angular_z = self.gain_yaw * roll_delta

        if self.invert_x:
            linear_x *= -1.0
        if self.invert_y:
            linear_y *= -1.0
        if self.invert_yaw:
            angular_z *= -1.0

        cmd = Twist()
        cmd.linear.x = self.clamp(linear_x, self.max_linear_x)
        cmd.linear.y = self.clamp(linear_y, self.max_linear_y)
        cmd.angular.z = self.clamp(angular_z, self.max_angular_z)
        return cmd

    def is_timeout(self):
        if self.last_msg_time is None:
            return True
        elapsed_sec = (self.get_clock().now() - self.last_msg_time).nanoseconds * 1e-9
        return elapsed_sec >= self.timeout_sec

    def publish_zero(self):
        self.cmd_pub.publish(Twist())

    def log_status(self, timeout):
        now = self.get_clock().now()
        elapsed_sec = (now - self.last_log_time).nanoseconds * 1e-9
        if elapsed_sec < 1.0:
            return
        self.last_log_time = now
        self.get_logger().info(
            f'active={self.active} timeout={timeout} '
            f'delta_x={self.delta_x:.4f} delta_z={self.delta_z:.4f} '
            f'roll_delta={self.roll_delta:.4f} '
            f'cmd linear.x={self.last_cmd.linear.x:.4f} '
            f'linear.y={self.last_cmd.linear.y:.4f} '
            f'angular.z={self.last_cmd.angular.z:.4f}'
        )

    @staticmethod
    def apply_deadzone(value, deadzone):
        if abs(value) < deadzone:
            return 0.0
        return value

    @staticmethod
    def clamp(value, max_abs):
        return max(-max_abs, min(max_abs, value))


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = AmirRightHandMecanumNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_zero()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
