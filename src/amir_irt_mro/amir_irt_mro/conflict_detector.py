#!/usr/bin/env python3
"""競合検出ノード"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
import numpy as np

CONFLICT_DIST_THRESHOLD = 0.15

class ConflictDetectorNode(Node):
    def __init__(self):
        super().__init__('conflict_detector')
        self.bottle_a = None
        self.bottle_b = None
        self.create_subscription(Float32MultiArray, '/user_a/identified_bottle_pose', self.bottle_a_cb, 10)
        self.create_subscription(Float32MultiArray, '/user_b/identified_bottle_pose', self.bottle_b_cb, 10)
        self.conflict_pub = self.create_publisher(String, '/conflict_alert', 10)
        self.priority_pub = self.create_publisher(String, '/priority_user', 10)
        self.create_timer(1.0/30.0, self.detect)
        self.get_logger().info('ConflictDetectorNode started')

    def bottle_a_cb(self, msg):
        if len(msg.data) >= 3:
            self.bottle_a = np.array(msg.data[:3])

    def bottle_b_cb(self, msg):
        if len(msg.data) >= 3:
            self.bottle_b = np.array(msg.data[:3])

    def detect(self):
        if self.bottle_a is None or self.bottle_b is None:
            return
        dist = np.linalg.norm(self.bottle_a - self.bottle_b)
        alert_msg = String()
        priority_msg = String()
        if dist < CONFLICT_DIST_THRESHOLD:
            alert_msg.data = 'CONFLICT'
            score_a = self._calc_score(self.bottle_a)
            score_b = self._calc_score(self.bottle_b)
            if abs(score_a - score_b) > 0.1:
                priority_msg.data = 'A' if score_a > score_b else 'B'
            else:
                priority_msg.data = 'ASK_USER'
        else:
            alert_msg.data = 'CLEAR'
            priority_msg.data = 'NONE'
        self.conflict_pub.publish(alert_msg)
        self.priority_pub.publish(priority_msg)

    def _calc_score(self, bottle_xyz):
        return 1.0 / (np.linalg.norm(bottle_xyz) + 1e-6)

def main(args=None):
    rclpy.init(args=args)
    node = ConflictDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
