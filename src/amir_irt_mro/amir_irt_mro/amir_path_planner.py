#!/usr/bin/env python3
"""
経路計画ノード（ROS2 Humble / Amir対応）
元：get_path_client.py（ROS1）をROS2に移植
機能：
  1. 複数オブジェクトの最適把持位置計算（凸包）
  2. 経路のウェイポイント変換
  3. 障害物回避の安全位置探索
"""
import math
import numpy as np
from scipy.spatial import ConvexHull

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path, OccupancyGrid
from std_msgs.msg import Float32MultiArray, Bool
import tf2_ros
from tf_transformations import euler_from_quaternion, quaternion_from_euler


class NearestSafePositionFinder:
    """障害物を避けた安全位置を探索"""
    def __init__(self, safety_radius=0.2):
        self.safety_radius = safety_radius
        self.map_data = None

    def set_map(self, map_data):
        self.map_data = map_data

    def is_safe_position(self, point):
        if self.map_data is None:
            return False
        resolution = self.map_data.info.resolution
        origin = self.map_data.info.origin
        min_x = int((point.x - self.safety_radius - origin.position.x) / resolution)
        max_x = int((point.x + self.safety_radius - origin.position.x) / resolution)
        min_y = int((point.y - self.safety_radius - origin.position.y) / resolution)
        max_y = int((point.y + self.safety_radius - origin.position.y) / resolution)
        for x in range(min_x, max_x + 1):
            for y in range(min_y, max_y + 1):
                if (0 <= x < self.map_data.info.width and
                        0 <= y < self.map_data.info.height):
                    idx = y * self.map_data.info.width + x
                    if self.map_data.data[idx] == 100:
                        return False
        return True

    def get_nearest_safe_position(self, target, origin, move_distance=0.01):
        target_point = Point(x=target[0], y=target[1], z=0.0)
        if self.is_safe_position(target_point):
            return target_point
        directions = [(0, 1)]
        nearest = target_point
        min_dist = 0.0
        for i in range(50):
            for dx, dy in directions:
                candidate = Point(
                    x=target[0] + dx * move_distance * (i + 1),
                    y=target[1] + dy * move_distance * (i + 1),
                    z=0.0)
                if self.is_safe_position(candidate):
                    dist = math.sqrt(
                        (candidate.x - origin.x) ** 2 +
                        (candidate.y - origin.y) ** 2)
                    if dist > min_dist:
                        min_dist = dist
                        nearest = candidate
        return nearest


def optimal_position(current_pose, pick_pos_data):
    """複数オブジェクトの最適把持位置を凸包で計算"""
    initial = np.array([
        current_pose.pose.position.x,
        current_pose.pose.position.y])
    pick_list = []
    for i in range(int(len(pick_pos_data) / 3)):
        pick_list.append([
            -pick_pos_data[i * 3],
            -pick_pos_data[i * 3 + 2]])
    obj_pos = np.array(pick_list)
    max_reach = 0.5
    safety_dist = 0.3
    hull = ConvexHull(obj_pos)
    hull_centroid = np.mean(obj_pos[hull.vertices], axis=0)
    optimal = hull_centroid
    min_dist = np.inf
    for angle in np.linspace(0, 2 * np.pi, 100):
        for r in np.linspace(safety_dist, max_reach - safety_dist, 50):
            candidate = hull_centroid + r * np.array([
                np.cos(angle), np.sin(angle)])
            distances = np.linalg.norm(obj_pos - candidate, axis=1)
            if np.all((distances <= max_reach) & (distances >= safety_dist)):
                d = np.linalg.norm(candidate - initial)
                if d < min_dist:
                    min_dist = d
                    optimal = candidate
    return optimal


def process_path(path):
    """経路を0.1m間隔と曲がり角でウェイポイントに変換"""
    waypoints = []
    accumulated = 0.0
    poses = path.poses
    for i in range(1, len(poses), 5):
        if i == 1:
            waypoints.append(poses[i - 1])
            continue
        dx = poses[i].pose.position.x - poses[i-1].pose.position.x
        dy = poses[i].pose.position.y - poses[i-1].pose.position.y
        dist = math.sqrt(dx**2 + dy**2)
        accumulated += dist
        if accumulated >= 0.1:
            waypoints.append(poses[i])
            accumulated = 0.0
    waypoints.append(poses[-1])
    return waypoints


class AmirPathPlannerNode(Node):
    def __init__(self):
        super().__init__('amir_path_planner')

        self.finder = NearestSafePositionFinder(safety_radius=0.2)
        self.pick_object_pose = None
        self.current_pose = None

        # TF
        self.tf_buf = tf2_ros.Buffer()
        self.tf_lstn = tf2_ros.TransformListener(self.tf_buf, self)

        # Subscriber
        self.create_subscription(
            Float32MultiArray,
            '/Pick_Object_Command',
            self.pick_object_cb, 10)
        self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_cb, 10)

        # Publisher
        self.plan_pub = self.create_publisher(Path, '/IRM_Edit_Route', 10)
        self.done_pub = self.create_publisher(Bool, '/plan_done', 10)

        # 10Hzで計画
        self.create_timer(0.1, self.plan_loop)
        self.plan_done = False

        self.get_logger().info('AmirPathPlannerNode started')

    def pick_object_cb(self, msg):
        self.pick_object_pose = msg.data
        self.plan_done = False

    def map_cb(self, msg):
        self.finder.set_map(msg)

    def get_current_pose(self):
        try:
            trans = self.tf_buf.lookup_transform(
                'map', 'base_link', rclpy.time.Time())
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = trans.transform.translation.x
            pose.pose.position.y = trans.transform.translation.y
            pose.pose.position.z = trans.transform.translation.z
            pose.pose.orientation = trans.transform.rotation
            return pose
        except Exception as e:
            self.get_logger().warn(f'TF lookup failed: {e}')
            return None

    def plan_loop(self):
        if self.plan_done:
            return
        if self.pick_object_pose is None or len(self.pick_object_pose) == 0:
            return

        current = self.get_current_pose()
        if current is None:
            return

        # 最適位置計算
        opt_pos = optimal_position(current, self.pick_object_pose)
        origin_pt = Point(
            x=current.pose.position.x,
            y=current.pose.position.y,
            z=0.0)
        safe_pos = self.finder.get_nearest_safe_position(
            opt_pos, origin_pt)

        self.get_logger().info(
            f'Optimal position: ({safe_pos.x:.2f}, {safe_pos.y:.2f})')

        # 経路生成（直線経路）
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()

        # 現在位置→目標位置の直線を10点で補間
        steps = 10
        for i in range(steps + 1):
            t = i / steps
            ps = PoseStamped()
            ps.header.frame_id = 'map'
            ps.pose.position.x = (
                current.pose.position.x * (1 - t) + safe_pos.x * t)
            ps.pose.position.y = (
                current.pose.position.y * (1 - t) + safe_pos.y * t)
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)

        waypoints = process_path(path)
        path.poses = waypoints
        self.plan_pub.publish(path)
        self.plan_done = True
        self.get_logger().info('Path published.')


def main(args=None):
    rclpy.init(args=args)
    node = AmirPathPlannerNode()
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
