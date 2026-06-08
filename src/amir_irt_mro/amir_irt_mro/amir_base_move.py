#!/usr/bin/env python3
"""
IRMベース移動ノード（ROS2 Humble / Amir対応）
元：IRM_youbot_baseMove_az.py（ROS1）をROS2に移植
"""
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import tf2_ros
from tf_transformations import euler_from_quaternion


class AmirBaseMoveNode(Node):
    def __init__(self):
        super().__init__('amir_base_move')

        self.declare_parameter('route_topic', '/IRM_Edit_Route')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('tolerance', 0.1)
        self.declare_parameter('max_lin_vel', 0.3)
        self.declare_parameter('max_ang_vel', 0.5)

        self.route_topic = self.get_parameter('route_topic').value
        self.cmd_topic   = self.get_parameter('cmd_vel_topic').value
        self.frame_map   = self.get_parameter('map_frame').value
        self.frame_base  = self.get_parameter('base_frame').value
        self.waypoint_tolerance = self.get_parameter('tolerance').value
        self.max_lin_vel = self.get_parameter('max_lin_vel').value

        self.cmd_pub  = self.create_publisher(Twist, self.cmd_topic, 1)
        self.done_pub = self.create_publisher(Bool, '/route_done', 1)
        self.create_subscription(Path, self.route_topic, self.route_callback, 1)

        self.tf_buf  = tf2_ros.Buffer()
        self.tf_lstn = tf2_ros.TransformListener(self.tf_buf, self)

        self.is_running = False
        self.waypoints = []
        self.current_wp_idx = 0

        self.create_timer(0.1, self.control_loop)
        self.get_logger().info(f'AmirBaseMoveNode started. Listening on {self.route_topic}')

    def route_callback(self, msg: Path):
        if self.is_running:
            self.get_logger().warn('Already running. Ignoring new route.')
            return
        self.get_logger().info(f'Received route: {len(msg.poses)} waypoints')
        self.waypoints = [
            (ps.pose.position.x, ps.pose.position.y, ps.pose.orientation)
            for ps in msg.poses
        ]
        self.current_wp_idx = 0
        self.is_running = True

    def control_loop(self):
        if not self.is_running or not self.waypoints:
            return
        if self.current_wp_idx >= len(self.waypoints):
            self.stop()
            done_msg = Bool()
            done_msg.data = True
            self.done_pub.publish(done_msg)
            self.is_running = False
            self.get_logger().info('Route completed.')
            return

        wx, wy, _ = self.waypoints[self.current_wp_idx]
        try:
            trans = self.tf_buf.lookup_transform(
                self.frame_map, self.frame_base, rclpy.time.Time())
        except Exception as e:
            self.get_logger().warn(f'TF lookup failed: {e}')
            return

        rx = trans.transform.translation.x
        ry = trans.transform.translation.y
        q  = trans.transform.rotation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

        dx = wx - rx
        dy = wy - ry
        dist = math.hypot(dx, dy)

        if dist < self.waypoint_tolerance:
            self.get_logger().info(f'Reached waypoint {self.current_wp_idx}')
            self.current_wp_idx += 1
            return

        vx_map = dx / dist * self.max_lin_vel
        vy_map = dy / dist * self.max_lin_vel
        vel = Twist()
        vel.linear.x =  math.cos(yaw) * vx_map + math.sin(yaw) * vy_map
        vel.linear.y = -math.sin(yaw) * vx_map + math.cos(yaw) * vy_map
        vel.angular.z = 0.0
        self.cmd_pub.publish(vel)

    def stop(self):
        self.cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = AmirBaseMoveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
