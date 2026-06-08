#!/usr/bin/env python3
"""情報フィルタリングノード"""
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped
import numpy as np

PROXIMITY_THRESHOLD = 0.5

class InfoFilterNode(Node):
    def __init__(self):
        super().__init__('info_filter')
        self.pose_a = None
        self.pose_b = None
        self.conflict = 'CLEAR'
        self.task_phase = 'STEP1'
        self.create_subscription(PoseStamped, '/amir1/base_pose', self.pose_a_cb, 10)
        self.create_subscription(PoseStamped, '/amir2/base_pose', self.pose_b_cb, 10)
        self.create_subscription(String, '/conflict_alert', self.conflict_cb, 10)
        self.create_subscription(String, '/task_phase', self.phase_cb, 10)
        self.filter_a_pub = self.create_publisher(String, '/user_a/display_info', 10)
        self.filter_b_pub = self.create_publisher(String, '/user_b/display_info', 10)
        self.handover_pub = self.create_publisher(Bool, '/handover_ready', 10)
        self.create_timer(1.0/10.0, self.filter)
        self.get_logger().info('InfoFilterNode started')

    def pose_a_cb(self, msg):
        self.pose_a = np.array([msg.pose.position.x, msg.pose.position.y])

    def pose_b_cb(self, msg):
        self.pose_b = np.array([msg.pose.position.x, msg.pose.position.y])

    def conflict_cb(self, msg):
        self.conflict = msg.data

    def phase_cb(self, msg):
        self.task_phase = msg.data

    def filter(self):
        if self.pose_a is None or self.pose_b is None:
            return
        dist = np.linalg.norm(self.pose_a - self.pose_b)
        near = dist < PROXIMITY_THRESHOLD
        for viewer, pub in [('A', self.filter_a_pub), ('B', self.filter_b_pub)]:
            info = {
                'show_partner_full': near,
                'show_partner_pos': True,
                'show_conflict': self.conflict == 'CONFLICT',
                'show_handover': self.task_phase == 'STEP3',
                'phase': self.task_phase,
            }
            msg = String()
            msg.data = json.dumps(info)
            pub.publish(msg)
        handover_msg = Bool()
        handover_msg.data = (self.task_phase == 'STEP3')
        self.handover_pub.publish(handover_msg)

def main(args=None):
    rclpy.init(args=args)
    node = InfoFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
