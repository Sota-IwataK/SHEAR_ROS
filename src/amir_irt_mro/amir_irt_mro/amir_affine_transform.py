#!/usr/bin/env python3
"""
アフィン変換ノード（ROS2 Humble / Amir対応）
元：afine_transformation.py（ROS1）をROS2に移植
機能：MR空間とロボット空間の座標変換
"""
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


def affine_transformation(before, after, obj):
    """
    4点対応によるアフィン変換行列を求め、objの座標を変換する
    before: 変換前の4点座標（Float32MultiArray）
    after:  変換後の4点座標（Float32MultiArray）
    obj:    変換したい座標群（Float32MultiArray）
    """
    x_before, y_before, z_before = [], [], []
    x_after,  y_after,  z_after  = [], [], []

    for i in range(int(len(before.data) / 3)):
        x_before.append(before.data[i*3]     - before.data[0])
        y_before.append(before.data[i*3 + 1] - before.data[1])
        z_before.append(before.data[i*3 + 2] - before.data[2])

    for j in range(int(len(after.data) / 3)):
        x_after.append(after.data[j*3]     - after.data[0])
        y_after.append(after.data[j*3 + 1] - after.data[1])
        z_after.append(after.data[j*3 + 2] - after.data[2])

    dst = np.array([
        x_after[0], y_after[0], z_after[0],
        x_after[1], y_after[1], z_after[1],
        x_after[2], y_after[2], z_after[2],
        x_after[3], y_after[3], z_after[3],
    ]).T

    before_list = []
    for i in range(4):
        before_list.append([x_before[i], y_before[i], z_before[i], 1, 0, 0, 0, 0, 0, 0, 0, 0])
        before_list.append([0, 0, 0, 0, x_before[i], y_before[i], z_before[i], 1, 0, 0, 0, 0])
        before_list.append([0, 0, 0, 0, 0, 0, 0, 0, x_before[i], y_before[i], z_before[i], 1])

    mat = np.array(before_list)
    ans = np.matmul(np.linalg.pinv(mat), dst)
    affine = np.array([
        [ans[0], ans[1],  ans[2],  ans[3]],
        [ans[4], ans[5],  ans[6],  ans[7]],
        [ans[8], ans[9],  ans[10], ans[11]],
        [0,      0,       0,       1     ],
    ])

    result = []
    for k in range(int(len(obj.data) / 3)):
        ox = obj.data[k*3]     - before.data[0]
        oy = obj.data[k*3 + 1] - before.data[1]
        oz = obj.data[k*3 + 2] - before.data[2]
        transformed = np.dot(affine, np.array([ox, oy, oz, 1]))
        result.append(transformed[0] + after.data[0])
        result.append(transformed[1] + after.data[1])
        result.append(transformed[2] + after.data[2])

    return result


class AmirAffineTransformNode(Node):
    def __init__(self):
        super().__init__('amir_affine_transform')

        self.before_pose = Float32MultiArray()
        self.after_pose  = Float32MultiArray()
        self.obj_pose    = Float32MultiArray()

        # Subscriber
        self.create_subscription(
            Float32MultiArray, '/before_command',
            self.before_cb, 10)
        self.create_subscription(
            Float32MultiArray, '/after_command',
            self.after_cb, 10)
        self.create_subscription(
            Float32MultiArray, '/calibration_command',
            self.obj_cb, 10)

        # Publisher
        self.pub = self.create_publisher(
            Float32MultiArray, '/after_calibration_command', 10)

        self.create_timer(0.1, self.transform_loop)
        self.get_logger().info('AmirAffineTransformNode started')

    def before_cb(self, msg):
        self.before_pose = msg

    def after_cb(self, msg):
        self.after_pose = msg

    def obj_cb(self, msg):
        self.obj_pose = msg

    def transform_loop(self):
        if len(self.before_pose.data) <= 2:
            return
        if len(self.after_pose.data) <= 2:
            return
        if len(self.obj_pose.data) == 0:
            return

        try:
            result = affine_transformation(
                self.before_pose, self.after_pose, self.obj_pose)
            msg = Float32MultiArray(data=result)
            self.pub.publish(msg)
        except Exception as e:
            self.get_logger().warn(f'Affine transform failed: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = AmirAffineTransformNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
