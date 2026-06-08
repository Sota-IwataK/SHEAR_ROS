#!/usr/bin/env python3

import math
import threading

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image


class RealSenseBottlePoseNode(Node):
    def __init__(self):
        super().__init__("realsense_bottle_pose_node")

        self.declare_parameter("color_topic", "/camera/camera/color/image_raw")
        self.declare_parameter(
            "depth_topic", "/camera/camera/aligned_depth_to_color/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera/color/camera_info")
        self.declare_parameter("output_topic", "/detected_bottle_pose")
        self.declare_parameter("pose_array_topic", "/detected_bottle_poses")
        self.declare_parameter("frame_id", "camera_color_optical_frame")
        self.declare_parameter("yolo_model", "yolov8n.pt")
        self.declare_parameter("confidence_threshold", 0.25)
        self.declare_parameter("depth_window_px", 5)
        self.declare_parameter("max_bottles", 5)
        self.declare_parameter("publish_pose_array", True)
        self.declare_parameter("max_depth_m", 1.2)
        self.declare_parameter("min_depth_m", 0.05)

        self.color_topic = str(self.get_parameter("color_topic").value)
        self.depth_topic = str(self.get_parameter("depth_topic").value)
        self.camera_info_topic = str(self.get_parameter("camera_info_topic").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.pose_array_topic = str(self.get_parameter("pose_array_topic").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.yolo_model_path = str(self.get_parameter("yolo_model").value)
        self.confidence_threshold = float(
            self.get_parameter("confidence_threshold").value)
        self.depth_window_px = max(1, int(self.get_parameter("depth_window_px").value))
        if self.depth_window_px % 2 == 0:
            self.depth_window_px += 1
        self.max_bottles = max(1, int(self.get_parameter("max_bottles").value))
        self.publish_pose_array = bool(self.get_parameter("publish_pose_array").value)
        self.max_depth_m = float(self.get_parameter("max_depth_m").value)
        self.min_depth_m = max(0.0, float(self.get_parameter("min_depth_m").value))
        if self.max_depth_m < self.min_depth_m:
            self.get_logger().warn(
                "max_depth_m is smaller than min_depth_m. Swapping depth limits.")
            self.min_depth_m, self.max_depth_m = self.max_depth_m, self.min_depth_m

        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.latest_depth_m = None
        self.latest_camera_info = None

        try:
            from ultralytics import YOLO
            self.model = YOLO(self.yolo_model_path)
        except Exception as exc:
            self.model = None
            self.get_logger().error(
                "Failed to load ultralytics YOLO model "
                f"{self.yolo_model_path}: {exc}")

        self.pose_pub = self.create_publisher(PoseStamped, self.output_topic, 10)
        self.pose_array_pub = self.create_publisher(
            PoseArray, self.pose_array_topic, 10)
        self.create_subscription(Image, self.color_topic, self.color_cb, 10)
        self.create_subscription(Image, self.depth_topic, self.depth_cb, 10)
        self.create_subscription(
            CameraInfo, self.camera_info_topic, self.camera_info_cb, 10)

        self.get_logger().info(
            "RealSenseBottlePoseNode started "
            f"color_topic={self.color_topic} "
            f"depth_topic={self.depth_topic} "
            f"camera_info_topic={self.camera_info_topic} "
            f"output_topic={self.output_topic} "
            f"pose_array_topic={self.pose_array_topic} "
            f"frame_id={self.frame_id} "
            f"yolo_model={self.yolo_model_path} "
            f"confidence_threshold={self.confidence_threshold:.3f} "
            f"depth_window_px={self.depth_window_px} "
            f"max_bottles={self.max_bottles} "
            f"publish_pose_array={self.publish_pose_array} "
            f"min_depth_m={self.min_depth_m:.3f} "
            f"max_depth_m={self.max_depth_m:.3f}")

    def camera_info_cb(self, msg):
        with self.lock:
            self.latest_camera_info = msg

    def depth_cb(self, msg):
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as exc:
            self.get_logger().warn(f"Failed to convert depth image: {exc}")
            return

        depth = np.asarray(depth)
        if msg.encoding in ("16UC1", "mono16"):
            depth_m = depth.astype(np.float32) * 0.001
        else:
            depth_m = depth.astype(np.float32)

        depth_m[~np.isfinite(depth_m)] = np.nan
        with self.lock:
            self.latest_depth_m = depth_m

    def color_cb(self, msg):
        if self.model is None:
            self.get_logger().warn(
                "YOLO model is not available; skipping color frame.",
                throttle_duration_sec=5.0)
            return

        with self.lock:
            depth_m = None if self.latest_depth_m is None else self.latest_depth_m.copy()
            camera_info = self.latest_camera_info

        if depth_m is None or camera_info is None:
            self.get_logger().warn(
                "Waiting for aligned depth image and camera_info.",
                throttle_duration_sec=2.0)
            return

        try:
            color_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().warn(f"Failed to convert color image: {exc}")
            return

        detections = self.detect_bottles(color_bgr)
        if not detections:
            return

        valid_bottles = []
        for conf, u, v in detections:
            depth = self.depth_median_at(depth_m, u, v)
            if depth is None:
                self.get_logger().warn(
                    "Bottle depth invalid at bbox center "
                    f"u={u} v={v}; skipping this detection.")
                continue
            if depth < self.min_depth_m or depth > self.max_depth_m:
                self.get_logger().info(
                    "bottle skipped by depth range "
                    f"conf={conf:.3f} depth_m={depth:.4f} "
                    f"pixel=[{u}, {v}] "
                    f"min_depth_m={self.min_depth_m:.3f} "
                    f"max_depth_m={self.max_depth_m:.3f}")
                continue

            xyz = self.deproject_pixel_to_point(camera_info, u, v, depth)
            if xyz is None:
                continue
            valid_bottles.append((conf, depth, xyz, u, v))
            if len(valid_bottles) >= self.max_bottles:
                break

        if not valid_bottles:
            return

        pose_array_msg = PoseArray()
        pose_array_msg.header.stamp = msg.header.stamp
        pose_array_msg.header.frame_id = self.frame_id
        for conf, depth, xyz, u, v in valid_bottles:
            pose_array_msg.poses.append(self.make_pose(xyz))

        if self.publish_pose_array:
            self.pose_array_pub.publish(pose_array_msg)

        pose_msg = PoseStamped()
        pose_msg.header.stamp = msg.header.stamp
        pose_msg.header.frame_id = self.frame_id
        pose_msg.pose = pose_array_msg.poses[0]
        self.pose_pub.publish(pose_msg)

        self.get_logger().info(f"detected valid bottles count={len(valid_bottles)}")
        for index, (conf, depth, xyz, u, v) in enumerate(valid_bottles):
            self.get_logger().info(
                "bottle detected "
                f"index={index} conf={conf:.3f} depth_m={depth:.4f} "
                f"xyz_camera=[{xyz[0]:.4f}, {xyz[1]:.4f}, {xyz[2]:.4f}] "
                f"pixel=[{u}, {v}]")

    def detect_bottles(self, color_bgr):
        results = self.model.predict(
            color_bgr,
            conf=self.confidence_threshold,
            verbose=False)
        if not results:
            return []

        detections = []
        names = getattr(self.model, "names", {})
        for result in results:
            boxes = getattr(result, "boxes", None)
            if boxes is None:
                continue
            for box in boxes:
                cls_id = int(box.cls[0])
                label = str(names.get(cls_id, cls_id))
                if label != "bottle":
                    continue
                conf = float(box.conf[0])
                if conf < self.confidence_threshold:
                    continue
                xyxy = box.xyxy[0].detach().cpu().numpy().astype(float)
                u = int(round(0.5 * (xyxy[0] + xyxy[2])))
                v = int(round(0.5 * (xyxy[1] + xyxy[3])))
                detections.append((conf, u, v))
        detections.sort(key=lambda item: item[0], reverse=True)
        return detections

    def make_pose(self, xyz):
        pose = Pose()
        pose.position.x = float(xyz[0])
        pose.position.y = float(xyz[1])
        pose.position.z = float(xyz[2])
        pose.orientation.w = 1.0
        return pose

    def depth_median_at(self, depth_m, u, v):
        height, width = depth_m.shape[:2]
        if u < 0 or u >= width or v < 0 or v >= height:
            return None

        half = self.depth_window_px // 2
        x0 = max(0, u - half)
        x1 = min(width, u + half + 1)
        y0 = max(0, v - half)
        y1 = min(height, v + half + 1)
        patch = depth_m[y0:y1, x0:x1].reshape(-1)
        valid = patch[np.isfinite(patch) & (patch > 0.0)]
        if valid.size == 0:
            return None
        depth = float(np.median(valid))
        if not math.isfinite(depth) or depth <= 0.0:
            return None
        return depth

    def deproject_pixel_to_point(self, camera_info, u, v, depth):
        if len(camera_info.k) < 6:
            self.get_logger().warn("camera_info K matrix is invalid.")
            return None
        fx = float(camera_info.k[0])
        fy = float(camera_info.k[4])
        cx = float(camera_info.k[2])
        cy = float(camera_info.k[5])
        if abs(fx) < 1e-9 or abs(fy) < 1e-9:
            self.get_logger().warn(
                f"camera_info focal length is invalid fx={fx} fy={fy}.")
            return None

        x = (float(u) - cx) * depth / fx
        y = (float(v) - cy) * depth / fy
        z = depth
        return np.array([x, y, z], dtype=np.float64)


def main(args=None):
    rclpy.init(args=args)
    node = RealSenseBottlePoseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
