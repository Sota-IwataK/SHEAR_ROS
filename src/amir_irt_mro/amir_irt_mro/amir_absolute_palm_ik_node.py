#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Absolute palm-position EE IK controller for Amir."""

import math

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty, Float32MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


DEFAULT_JOINT_NAMES = ["Joint_1", "Joint_2", "Joint_3", "Joint_4", "Joint_5"]

L1_OFFSET = 0.248
LINK_LENGTHS = np.array([0.310, 0.310, 0.148], dtype=np.float64)

DEFAULT_JOINTS = np.array([0.0, 1.3535, -2.5596, 0.5270, 0.0], dtype=np.float64)

JOINT_LIMITS = np.array([
    [-2.96706, 2.96706],
    [0.0, 2.356194],
    [-2.792527, 0.0],
    [-2.094395, 1.308997],
    [-2.96706, 2.96706],
], dtype=np.float64)


def clamp_array(value, lower, upper):
    return np.minimum(np.maximum(value, lower), upper)


def seconds_now(node):
    return node.get_clock().now().nanoseconds * 1e-9


def duration_from_seconds(value):
    sec = int(math.floor(max(value, 0.0)))
    nanosec = int(round((max(value, 0.0) - sec) * 1_000_000_000))
    if nanosec >= 1_000_000_000:
        sec += 1
        nanosec -= 1_000_000_000
    return Duration(sec=sec, nanosec=nanosec)


def as_bool(value):
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def normalized(value, fallback):
    norm = float(np.linalg.norm(value))
    if norm <= 1e-9:
        return fallback.copy(), False
    return value / norm, True


def basis_from_three_points(p0, p1, p2):
    x_axis, x_ok = normalized(p1 - p0, np.array([1.0, 0.0, 0.0], dtype=np.float64))
    y_raw = p2 - p0
    y_axis = y_raw - float(np.dot(y_raw, x_axis)) * x_axis
    y_axis, y_ok = normalized(y_axis, np.array([0.0, 1.0, 0.0], dtype=np.float64))
    z_axis = np.cross(x_axis, y_axis)
    z_axis, z_ok = normalized(z_axis, np.array([0.0, 0.0, 1.0], dtype=np.float64))
    y_axis = np.cross(z_axis, x_axis)
    y_axis, y2_ok = normalized(y_axis, np.array([0.0, 1.0, 0.0], dtype=np.float64))
    return np.column_stack((x_axis, y_axis, z_axis)), x_ok and y_ok and z_ok and y2_ok


def fk_xyz(joints):
    """Amir arm FK for Joint_1..Joint_5 position, base frame, meters."""
    j1, j2, j3, j4, _j5 = [float(v) for v in joints]
    th0 = math.pi / 2.0 - j2
    th1 = j3
    th2 = j4

    r = (LINK_LENGTHS[0] * math.sin(th0) +
         LINK_LENGTHS[1] * math.sin(th0 + th1) +
         LINK_LENGTHS[2] * math.sin(th0 + th1 + th2))
    z = (L1_OFFSET +
         LINK_LENGTHS[0] * math.cos(th0) +
         LINK_LENGTHS[1] * math.cos(th0 + th1) +
         LINK_LENGTHS[2] * math.cos(th0 + th1 + th2))
    x = -r * math.cos(j1)
    y = -r * math.sin(j1)
    return np.array([x, y, z], dtype=np.float64)


def jacobian(joints, eps=1e-5):
    base = fk_xyz(joints)
    jac = np.zeros((3, 5), dtype=np.float64)
    for idx in range(5):
        shifted = joints.copy()
        shifted[idx] += eps
        jac[:, idx] = (fk_xyz(shifted) - base) / eps
    return jac


def dls_ik(target_xyz, seed_joints, ik_lambda, ik_step_scale, max_iters):
    joints = clamp_array(seed_joints.copy(), JOINT_LIMITS[:, 0], JOINT_LIMITS[:, 1])
    damping = max(float(ik_lambda), 0.0)
    step_scale = max(float(ik_step_scale), 0.0)

    for _ in range(max(int(max_iters), 0)):
        current = fk_xyz(joints)
        error = target_xyz - current
        if float(np.linalg.norm(error)) < 1e-4:
            break

        jac = jacobian(joints)
        lhs = jac.T @ jac + (damping * damping) * np.eye(5)
        rhs = jac.T @ error
        try:
            delta = np.linalg.solve(lhs, rhs)
        except np.linalg.LinAlgError:
            delta = np.linalg.pinv(lhs) @ rhs

        delta = np.clip(step_scale * delta, -0.08, 0.08)
        joints = clamp_array(joints + delta, JOINT_LIMITS[:, 0], JOINT_LIMITS[:, 1])

    final_error = float(np.linalg.norm(target_xyz - fk_xyz(joints)))
    return joints, final_error


class AmirAbsolutePalmIkNode(Node):
    def __init__(self):
        super().__init__("amir_absolute_palm_ik_node")

        self.declare_parameter("mode", "gazebo")
        self.declare_parameter("control_rate_hz", 30.0)
        self.declare_parameter("palm_mapping_mode", "axis_map")
        self.declare_parameter("palm_to_robot_axis_map", [2, 0, 1])
        self.declare_parameter("palm_to_robot_axis_sign", [1.0, 1.0, 1.0])
        self.declare_parameter("mr_calib_p0", [0.0, 0.0, 0.0])
        self.declare_parameter("mr_calib_p1", [0.0, 0.0, 1.0])
        self.declare_parameter("mr_calib_p2", [1.0, 0.0, 0.0])
        self.declare_parameter("robot_calib_p0", [0.0, 0.0, 0.0])
        self.declare_parameter("robot_calib_p1", [0.3, 0.0, 0.0])
        self.declare_parameter("robot_calib_p2", [0.0, 0.3, 0.0])
        self.declare_parameter("calibrated_scale", 1.0)
        self.declare_parameter("target_mapping_mode", "linear_xyz")
        self.declare_parameter("x_gain", 1.0)
        self.declare_parameter("x_offset", 0.25)
        self.declare_parameter("min_x", 0.20)
        self.declare_parameter("max_x", 0.75)
        self.declare_parameter("relative_x_scale", 1.8)
        self.declare_parameter("relative_y_scale", 0.8)
        self.declare_parameter("relative_z_scale", 0.4)
        self.declare_parameter("relative_deadband_m", 0.005)
        self.declare_parameter("relative_reset_on_start", True)
        self.declare_parameter("hmd_extend_scale", 2.5)
        self.declare_parameter("hmd_lateral_scale", 0.10)
        self.declare_parameter("hmd_depth_scale", 0.10)
        self.declare_parameter("hmd_deadband_m", 0.005)
        self.declare_parameter("absolute_mr_center_x", 0.0)
        self.declare_parameter("absolute_mr_center_y", 1.2)
        self.declare_parameter("absolute_mr_center_z", 0.5)
        self.declare_parameter("absolute_robot_center_x", 0.30)
        self.declare_parameter("absolute_robot_center_y", 0.0)
        self.declare_parameter("absolute_robot_center_z", 0.45)
        self.declare_parameter("absolute_scale_x", 1.2)
        self.declare_parameter("absolute_scale_y", 0.5)
        self.declare_parameter("absolute_scale_z", 2.0)
        self.declare_parameter("palm_x_min", 0.00)
        self.declare_parameter("palm_x_max", 0.13)
        self.declare_parameter("palm_y_min", -0.40)
        self.declare_parameter("palm_y_max", 0.40)
        self.declare_parameter("palm_z_min", 0.10)
        self.declare_parameter("palm_z_max", 0.70)
        self.declare_parameter("target_x_min", 0.25)
        self.declare_parameter("target_x_max", 0.72)
        self.declare_parameter("target_y_min", -0.20)
        self.declare_parameter("target_y_max", 0.20)
        self.declare_parameter("target_z_min", 0.40)
        self.declare_parameter("target_z_max", 0.58)
        self.declare_parameter("enable_reachability_guard", True)
        self.declare_parameter("guard_joint2_max", 2.30)
        self.declare_parameter("guard_error_max", 0.03)
        self.declare_parameter("guard_scale_candidates", [1.0, 0.95, 0.90, 0.85, 0.80, 0.75])
        self.declare_parameter("r_gain", 1.0)
        self.declare_parameter("y_gain", 1.0)
        self.declare_parameter("z_gain", 0.3)
        self.declare_parameter("r_offset", 0.0)
        self.declare_parameter("y_offset", 0.0)
        self.declare_parameter("z_offset", 0.0)
        self.declare_parameter("min_r", 0.05)
        self.declare_parameter("max_r", 0.75)
        self.declare_parameter("min_y", -0.30)
        self.declare_parameter("max_y", 0.30)
        self.declare_parameter("min_z", 0.25)
        self.declare_parameter("max_z", 0.70)
        self.declare_parameter("max_target_step_m", 0.03)
        self.declare_parameter("trajectory_time_from_start_sec", 0.35)
        self.declare_parameter("ik_lambda", 0.03)
        self.declare_parameter("ik_step_scale", 0.5)
        self.declare_parameter("ik_iters", 80)
        self.declare_parameter("joint_names", DEFAULT_JOINT_NAMES)

        self.mode = str(self.get_parameter("mode").value).strip().lower()
        if self.mode not in ("sim", "gazebo", "real"):
            self.get_logger().warn(f'Invalid mode "{self.mode}". Falling back to gazebo.')
            self.mode = "gazebo"

        self.control_rate_hz = max(float(self.get_parameter("control_rate_hz").value), 0.1)
        self.palm_mapping_mode = str(
            self.get_parameter("palm_mapping_mode").value).strip().lower()
        self.palm_to_robot_axis_map = np.array(
            self.get_parameter("palm_to_robot_axis_map").value, dtype=np.int64)
        self.palm_to_robot_axis_sign = np.array(
            self.get_parameter("palm_to_robot_axis_sign").value, dtype=np.float64)
        self.mr_calib_p0 = np.array(self.get_parameter("mr_calib_p0").value, dtype=np.float64)
        self.mr_calib_p1 = np.array(self.get_parameter("mr_calib_p1").value, dtype=np.float64)
        self.mr_calib_p2 = np.array(self.get_parameter("mr_calib_p2").value, dtype=np.float64)
        self.robot_calib_p0 = np.array(
            self.get_parameter("robot_calib_p0").value, dtype=np.float64)
        self.robot_calib_p1 = np.array(
            self.get_parameter("robot_calib_p1").value, dtype=np.float64)
        self.robot_calib_p2 = np.array(
            self.get_parameter("robot_calib_p2").value, dtype=np.float64)
        self.calibrated_scale = float(self.get_parameter("calibrated_scale").value)
        self.target_mapping_mode = str(
            self.get_parameter("target_mapping_mode").value).strip().lower()
        self.x_gain = float(self.get_parameter("x_gain").value)
        self.x_offset = float(self.get_parameter("x_offset").value)
        self.min_x = float(self.get_parameter("min_x").value)
        self.max_x = float(self.get_parameter("max_x").value)
        self.relative_scale_xyz = np.array([
            float(self.get_parameter("relative_x_scale").value),
            float(self.get_parameter("relative_y_scale").value),
            float(self.get_parameter("relative_z_scale").value),
        ], dtype=np.float64)
        self.relative_deadband_m = max(
            float(self.get_parameter("relative_deadband_m").value), 0.0)
        self.relative_reset_on_start = as_bool(
            self.get_parameter("relative_reset_on_start").value)
        self.hmd_extend_scale = float(self.get_parameter("hmd_extend_scale").value)
        self.hmd_lateral_scale = float(self.get_parameter("hmd_lateral_scale").value)
        self.hmd_depth_scale = float(self.get_parameter("hmd_depth_scale").value)
        self.hmd_deadband_m = max(float(self.get_parameter("hmd_deadband_m").value), 0.0)
        self.absolute_mr_center = np.array([
            float(self.get_parameter("absolute_mr_center_x").value),
            float(self.get_parameter("absolute_mr_center_y").value),
            float(self.get_parameter("absolute_mr_center_z").value),
        ], dtype=np.float64)
        self.absolute_robot_center = np.array([
            float(self.get_parameter("absolute_robot_center_x").value),
            float(self.get_parameter("absolute_robot_center_y").value),
            float(self.get_parameter("absolute_robot_center_z").value),
        ], dtype=np.float64)
        self.absolute_scale_xyz = np.array([
            float(self.get_parameter("absolute_scale_x").value),
            float(self.get_parameter("absolute_scale_y").value),
            float(self.get_parameter("absolute_scale_z").value),
        ], dtype=np.float64)
        self.absolute_r_robot_from_mr = np.array([
            [0.0, 0.0, 1.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
        ], dtype=np.float64)
        self.palm_x_min = float(self.get_parameter("palm_x_min").value)
        self.palm_x_max = float(self.get_parameter("palm_x_max").value)
        self.palm_y_min = float(self.get_parameter("palm_y_min").value)
        self.palm_y_max = float(self.get_parameter("palm_y_max").value)
        self.palm_z_min = float(self.get_parameter("palm_z_min").value)
        self.palm_z_max = float(self.get_parameter("palm_z_max").value)
        self.target_x_min = float(self.get_parameter("target_x_min").value)
        self.target_x_max = float(self.get_parameter("target_x_max").value)
        self.target_y_min = float(self.get_parameter("target_y_min").value)
        self.target_y_max = float(self.get_parameter("target_y_max").value)
        self.target_z_min = float(self.get_parameter("target_z_min").value)
        self.target_z_max = float(self.get_parameter("target_z_max").value)
        self.enable_reachability_guard = as_bool(
            self.get_parameter("enable_reachability_guard").value)
        self.guard_joint2_max = float(self.get_parameter("guard_joint2_max").value)
        self.guard_error_max = max(float(self.get_parameter("guard_error_max").value), 0.0)
        self.guard_scale_candidates = np.array(
            self.get_parameter("guard_scale_candidates").value, dtype=np.float64)
        self.r_gain = float(self.get_parameter("r_gain").value)
        self.y_gain = float(self.get_parameter("y_gain").value)
        self.z_gain = float(self.get_parameter("z_gain").value)
        self.r_offset = float(self.get_parameter("r_offset").value)
        self.y_offset = float(self.get_parameter("y_offset").value)
        self.z_offset = float(self.get_parameter("z_offset").value)
        self.min_r = float(self.get_parameter("min_r").value)
        self.max_r = float(self.get_parameter("max_r").value)
        self.min_y = float(self.get_parameter("min_y").value)
        self.max_y = float(self.get_parameter("max_y").value)
        self.min_z = float(self.get_parameter("min_z").value)
        self.max_z = float(self.get_parameter("max_z").value)
        self.max_target_step_m = max(float(self.get_parameter("max_target_step_m").value), 0.0)
        self.trajectory_time_from_start_sec = float(
            self.get_parameter("trajectory_time_from_start_sec").value)
        self.ik_lambda = max(float(self.get_parameter("ik_lambda").value), 0.0)
        self.ik_step_scale = max(float(self.get_parameter("ik_step_scale").value), 0.0)
        self.ik_iters = max(int(self.get_parameter("ik_iters").value), 0)
        self.joint_names = [str(v) for v in self.get_parameter("joint_names").value]

        self.validate_parameters()
        self.update_calibrated_transform()

        self.current_joints = DEFAULT_JOINTS.copy()
        self.last_target_xyz = None
        self.last_palm_xyz = None
        self.last_palm_time = None
        self.last_hmd_delta_xyz = None
        self.last_hmd_delta_time = None
        self.latest_palm_world_mr = None
        self.latest_palm_world_mr_time = None
        self.joint_state_received = False

        self.last_palm_abs_robot = np.zeros(3, dtype=np.float64)
        self.last_mapping_mode_flag = 0.0
        self.last_target_mapping_mode_flag = 2.0
        self.last_normalized_xyz = np.zeros(3, dtype=np.float64)
        self.last_target_raw_xyz = np.zeros(3, dtype=np.float64)
        self.last_target_guarded_xyz = np.zeros(3, dtype=np.float64)
        self.last_target_step_limited_xyz = np.zeros(3, dtype=np.float64)
        self.palm_anchor_xyz = None
        self.ee_anchor_xyz = None
        self.force_relative_anchor_reset = bool(self.relative_reset_on_start)
        self.last_palm_anchor_xyz = np.zeros(3, dtype=np.float64)
        self.last_ee_anchor_xyz = np.zeros(3, dtype=np.float64)
        self.last_palm_delta_xyz = np.zeros(3, dtype=np.float64)
        self.last_hmd_delta_debug_xyz = np.zeros(3, dtype=np.float64)
        self.last_hmd_extend = 0.0
        self.last_hmd_lateral = 0.0
        self.last_hmd_depth = 0.0
        self.last_palm_world_mr = np.zeros(3, dtype=np.float64)
        self.last_absolute_mapped_delta_robot = np.zeros(3, dtype=np.float64)
        self.last_reachability_guard_enabled = 0.0
        self.last_selected_guard_scale = 1.0
        self.last_guard_joint2_hit = 0.0
        self.last_guard_error_hit = 0.0
        self.last_guard_warn_time = -1.0e9
        self.last_r = 0.0
        self.last_yaw = 0.0
        self.last_q_current = DEFAULT_JOINTS.copy()
        self.last_q_solution = DEFAULT_JOINTS.copy()
        self.last_fk_before = fk_xyz(DEFAULT_JOINTS)
        self.last_fk_after = fk_xyz(DEFAULT_JOINTS)
        self.last_error_before = float("nan")
        self.last_error_after = float("nan")
        self.last_debug_log_time = 0.0
        self.last_hmd_delta_log_time = -1.0e9
        self.last_absolute_wait_warn_time = -1.0e9
        self.palm_world_receive_log_count = 0
        self.reset_count = 0
        self.last_reset_accepted = 0.0

        hmd_relative_qos = QoSProfile(depth=50)
        hmd_relative_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        palm_world_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10)
        self.palm_sub = self.create_subscription(PoseStamped, "/palm_pose", self.palm_cb, 50)
        self.hmd_relative_sub = self.create_subscription(
            PoseStamped,
            "/palm_pose_hmd_relative",
            self.hmd_relative_cb,
            hmd_relative_qos)
        self.palm_world_sub = self.create_subscription(
            PoseStamped,
            "/palm_pose_world",
            self.palm_world_cb,
            palm_world_qos)
        self.joint_state_sub = self.create_subscription(
            JointState, "/joint_states", self.joint_state_cb, 10)
        self.reset_relative_anchor_sub = self.create_subscription(
            Empty, "/amir_abs/reset_relative_anchor", self.reset_relative_anchor_cb, 10)

        self.trajectory_pub = None
        if self.mode in ("gazebo", "real"):
            self.trajectory_pub = self.create_publisher(
                JointTrajectory, "/arm_controller/joint_trajectory", 10)
        self.target_pub = self.create_publisher(Float32MultiArray, "/amir_abs/target_xyz", 10)
        self.current_ee_pub = self.create_publisher(
            Float32MultiArray, "/amir_abs/current_ee_xyz", 10)
        self.palm_abs_pub = self.create_publisher(
            Float32MultiArray, "/amir_abs/palm_abs_robot", 10)
        self.ik_debug_pub = self.create_publisher(Float32MultiArray, "/amir_abs/ik_debug", 10)
        self.palm_anchor_robot_pub = self.create_publisher(
            PoseStamped, "/amir_abs/palm_anchor_robot", 10)
        self.ee_anchor_robot_pub = self.create_publisher(
            PoseStamped, "/amir_abs/ee_anchor_robot", 10)
        self.palm_delta_robot_pub = self.create_publisher(
            Vector3Stamped, "/amir_abs/palm_delta_robot", 10)
        self.target_pose_robot_pub = self.create_publisher(
            PoseStamped, "/amir_abs/target_pose_robot", 10)

        self.timer = self.create_timer(1.0 / self.control_rate_hz, self.control_loop)
        self.get_logger().info(
            "AmirAbsolutePalmIkNode started "
            f"mode={self.mode} rate={self.control_rate_hz:.1f}Hz "
            f"palm_mapping_mode={self.palm_mapping_mode} "
            f"target_mapping_mode={self.target_mapping_mode} "
            f"axis_map={self.palm_to_robot_axis_map.tolist()} "
            f"axis_sign={self.palm_to_robot_axis_sign.tolist()} "
            f"calibrated_scale={self.calibrated_scale:.4f} "
            f"R_robot_from_mr={np.round(self.r_robot_from_mr, 4).tolist()} "
            f"t_robot_from_mr={np.round(self.t_robot_from_mr, 4).tolist()} "
            f"gains=[x:{self.x_gain:.3f}, r:{self.r_gain:.3f}, "
            f"y:{self.y_gain:.3f}, z:{self.z_gain:.3f}] "
            f"relative_scale={np.round(self.relative_scale_xyz, 4).tolist()} "
            f"relative_deadband_m={self.relative_deadband_m:.4f} "
            f"relative_reset_on_start={self.relative_reset_on_start} "
            f"hmd_extend_scale={self.hmd_extend_scale:.4f} "
            f"hmd_lateral_scale={self.hmd_lateral_scale:.4f} "
            f"hmd_depth_scale={self.hmd_depth_scale:.4f} "
            f"hmd_deadband_m={self.hmd_deadband_m:.4f} "
            f"absolute_mr_center={np.round(self.absolute_mr_center, 4).tolist()} "
            f"absolute_robot_center={np.round(self.absolute_robot_center, 4).tolist()} "
            f"absolute_scale_xyz={np.round(self.absolute_scale_xyz, 4).tolist()} "
            f"absolute_R_robot_from_mr={self.absolute_r_robot_from_mr.tolist()} "
            f"offsets=[x:{self.x_offset:.3f}, r:{self.r_offset:.3f}, "
            f"y:{self.y_offset:.3f}, z:{self.z_offset:.3f}] "
            f"x_range=[{self.min_x:.3f}, {self.max_x:.3f}] "
            f"palm_range=x[{self.palm_x_min:.3f}, {self.palm_x_max:.3f}] "
            f"y[{self.palm_y_min:.3f}, {self.palm_y_max:.3f}] "
            f"z[{self.palm_z_min:.3f}, {self.palm_z_max:.3f}] "
            f"target_range=x[{self.target_x_min:.3f}, {self.target_x_max:.3f}] "
            f"y[{self.target_y_min:.3f}, {self.target_y_max:.3f}] "
            f"z[{self.target_z_min:.3f}, {self.target_z_max:.3f}] "
            f"enable_reachability_guard={self.enable_reachability_guard} "
            f"guard_joint2_max={self.guard_joint2_max:.3f} "
            f"guard_error_max={self.guard_error_max:.3f} "
            f"guard_scale_candidates={self.guard_scale_candidates.tolist()} "
            f"r_range=[{self.min_r:.3f}, {self.max_r:.3f}] "
            f"y_range=[{self.min_y:.3f}, {self.max_y:.3f}] "
            f"z_range=[{self.min_z:.3f}, {self.max_z:.3f}] "
            f"max_target_step_m={self.max_target_step_m:.3f} "
            f"trajectory_time_from_start_sec={self.trajectory_time_from_start_sec:.3f} "
            f"ik_lambda={self.ik_lambda:.4f} ik_step_scale={self.ik_step_scale:.3f} "
            f"ik_iters={self.ik_iters} joint_names={self.joint_names}")

    def validate_parameters(self):
        if self.palm_mapping_mode not in ("axis_map", "calibrated_transform"):
            self.get_logger().warn(
                f'Invalid palm_mapping_mode "{self.palm_mapping_mode}". Using axis_map.')
            self.palm_mapping_mode = "axis_map"
        if self.target_mapping_mode not in (
                "direct_xyz", "cylindrical", "linear_xyz", "relative_ee",
                "hmd_relative_ee", "absolute_scaled_ee"):
            self.get_logger().warn(
                f'Invalid target_mapping_mode "{self.target_mapping_mode}". Using linear_xyz.')
            self.target_mapping_mode = "linear_xyz"
        if (self.palm_to_robot_axis_map.shape != (3,) or
                np.any(self.palm_to_robot_axis_map < 0) or
                np.any(self.palm_to_robot_axis_map > 2)):
            self.get_logger().warn(
                "palm_to_robot_axis_map must be length 3 with values 0..2. "
                "Using [2, 0, 1].")
            self.palm_to_robot_axis_map = np.array([2, 0, 1], dtype=np.int64)
        if self.palm_to_robot_axis_sign.shape != (3,):
            self.get_logger().warn(
                "palm_to_robot_axis_sign must be length 3. Using [1.0, 1.0, 1.0].")
            self.palm_to_robot_axis_sign = np.array([1.0, 1.0, 1.0], dtype=np.float64)
        if self.min_x > self.max_x:
            self.min_x, self.max_x = self.max_x, self.min_x
        if self.palm_x_min > self.palm_x_max:
            self.palm_x_min, self.palm_x_max = self.palm_x_max, self.palm_x_min
        if self.palm_y_min > self.palm_y_max:
            self.palm_y_min, self.palm_y_max = self.palm_y_max, self.palm_y_min
        if self.palm_z_min > self.palm_z_max:
            self.palm_z_min, self.palm_z_max = self.palm_z_max, self.palm_z_min
        if self.target_x_min > self.target_x_max:
            self.target_x_min, self.target_x_max = self.target_x_max, self.target_x_min
        if self.target_y_min > self.target_y_max:
            self.target_y_min, self.target_y_max = self.target_y_max, self.target_y_min
        if self.target_z_min > self.target_z_max:
            self.target_z_min, self.target_z_max = self.target_z_max, self.target_z_min
        if self.min_r > self.max_r:
            self.min_r, self.max_r = self.max_r, self.min_r
        if self.min_y > self.max_y:
            self.min_y, self.max_y = self.max_y, self.min_y
        if self.min_z > self.max_z:
            self.min_z, self.max_z = self.max_z, self.min_z
        if self.relative_scale_xyz.shape != (3,):
            self.get_logger().warn(
                "relative scale parameters must form length 3. Using [1.8, 0.8, 0.4].")
            self.relative_scale_xyz = np.array([1.8, 0.8, 0.4], dtype=np.float64)
        if len(self.joint_names) != 5:
            self.get_logger().warn(
                "joint_names must be length 5. Using Joint_1..Joint_5.")
            self.joint_names = DEFAULT_JOINT_NAMES.copy()
        for name in (
                "mr_calib_p0", "mr_calib_p1", "mr_calib_p2",
                "robot_calib_p0", "robot_calib_p1", "robot_calib_p2"):
            value = getattr(self, name)
            if value.shape != (3,):
                self.get_logger().warn(f"{name} must be length 3. Using [0.0, 0.0, 0.0].")
                setattr(self, name, np.zeros(3, dtype=np.float64))
        if self.guard_scale_candidates.size == 0:
            self.get_logger().warn(
                "guard_scale_candidates must not be empty. Using [1.0, 0.9, 0.8].")
            self.guard_scale_candidates = np.array([1.0, 0.9, 0.8], dtype=np.float64)
        self.guard_scale_candidates = np.clip(self.guard_scale_candidates, 0.0, 1.0)

    def update_calibrated_transform(self):
        mr_basis, mr_ok = basis_from_three_points(
            self.mr_calib_p0, self.mr_calib_p1, self.mr_calib_p2)
        robot_basis, robot_ok = basis_from_three_points(
            self.robot_calib_p0, self.robot_calib_p1, self.robot_calib_p2)
        if not mr_ok or not robot_ok:
            self.get_logger().warn(
                "Calibration points are degenerate. Using identity calibrated transform.")
            self.r_robot_from_mr = np.eye(3, dtype=np.float64)
        else:
            self.r_robot_from_mr = robot_basis @ mr_basis.T
        self.t_robot_from_mr = (
            self.robot_calib_p0 -
            self.calibrated_scale * (self.r_robot_from_mr @ self.mr_calib_p0)
        )

    def palm_cb(self, msg):
        self.last_palm_xyz = np.array([
            float(msg.pose.position.x),
            float(msg.pose.position.y),
            float(msg.pose.position.z),
        ], dtype=np.float64)
        self.last_palm_time = seconds_now(self)

    def hmd_relative_cb(self, msg):
        self.last_hmd_delta_xyz = np.array([
            float(msg.pose.position.x),
            float(msg.pose.position.y),
            float(msg.pose.position.z),
        ], dtype=np.float64)
        self.last_hmd_delta_time = seconds_now(self)
        if self.last_hmd_delta_time - self.last_hmd_delta_log_time >= 1.0:
            self.last_hmd_delta_log_time = self.last_hmd_delta_time
            self.get_logger().info(
                "Received hmd relative pose "
                f"delta={np.round(self.last_hmd_delta_xyz, 4).tolist()}")

    def palm_world_cb(self, msg):
        self.latest_palm_world_mr = np.array([
            float(msg.pose.position.x),
            float(msg.pose.position.y),
            float(msg.pose.position.z),
        ], dtype=np.float64)
        self.last_palm_world_mr = self.latest_palm_world_mr.copy()
        self.latest_palm_world_mr_time = seconds_now(self)
        if self.palm_world_receive_log_count < 5:
            self.palm_world_receive_log_count += 1
            self.get_logger().info(
                "[absolute_scaled_ee] received /palm_pose_world "
                f"p={np.round(self.latest_palm_world_mr, 4).tolist()}")

    def joint_state_cb(self, msg):
        name_to_idx = {name: idx for idx, name in enumerate(msg.name)}
        updated = self.current_joints.copy()
        for joint_idx, joint_name in enumerate(self.joint_names):
            if joint_name in name_to_idx and name_to_idx[joint_name] < len(msg.position):
                updated[joint_idx] = float(msg.position[name_to_idx[joint_name]])
        self.current_joints = clamp_array(updated, JOINT_LIMITS[:, 0], JOINT_LIMITS[:, 1])
        self.joint_state_received = True

    def reset_relative_anchor_cb(self, _msg):
        current_ee_xyz = fk_xyz(self.current_joints)
        self.get_logger().warn(
            f"[absolute_scaled_ee] reset_relative_anchor received "
            f"mode={self.target_mapping_mode} "
            f"latest_palm_world_mr={self.latest_palm_world_mr} "
            f"current_ee_xyz={current_ee_xyz}")

        if self.target_mapping_mode == "absolute_scaled_ee":
            if self.latest_palm_world_mr is None:
                self.get_logger().warn(
                    "[absolute_scaled_ee] reset ignored: latest_palm_world_mr is None")
                self.last_reset_accepted = 0.0
                return
            self.absolute_mr_center = self.latest_palm_world_mr.copy()
            self.last_palm_world_mr = self.latest_palm_world_mr.copy()
            self.absolute_robot_center = current_ee_xyz.copy()
            self.ee_anchor_xyz = current_ee_xyz.copy()
            self.last_target_xyz = current_ee_xyz.copy()
            self.last_target_raw_xyz = current_ee_xyz.copy()
            self.last_target_guarded_xyz = current_ee_xyz.copy()
            self.last_target_step_limited_xyz = current_ee_xyz.copy()
            self.force_relative_anchor_reset = False
            self.reset_count += 1
            self.last_reset_accepted = 1.0
            self.get_logger().warn(
                f"[absolute_scaled_ee] reset applied "
                f"absolute_mr_center={np.round(self.absolute_mr_center, 4).tolist()} "
                f"absolute_robot_center={np.round(self.absolute_robot_center, 4).tolist()}")
            return

        self.palm_anchor_xyz = None
        self.ee_anchor_xyz = None
        self.force_relative_anchor_reset = True
        self.reset_count += 1
        self.last_reset_accepted = 1.0
        self.get_logger().info("Received relative anchor reset request")

    def publish_float_array(self, publisher, values):
        msg = Float32MultiArray()
        msg.data = [float(v) for v in values]
        publisher.publish(msg)

    def make_pose_stamped(self, xyz):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "amir_base"
        msg.pose.position.x = float(xyz[0])
        msg.pose.position.y = float(xyz[1])
        msg.pose.position.z = float(xyz[2])
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0
        return msg

    def make_vector3_stamped(self, xyz):
        msg = Vector3Stamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "amir_base"
        msg.vector.x = float(xyz[0])
        msg.vector.y = float(xyz[1])
        msg.vector.z = float(xyz[2])
        return msg

    def palm_xyz_to_robot_abs(self, palm_xyz):
        if self.palm_mapping_mode == "calibrated_transform":
            self.last_mapping_mode_flag = 1.0
            return (
                self.robot_calib_p0 +
                self.calibrated_scale * (
                    self.r_robot_from_mr @ (palm_xyz - self.mr_calib_p0))
            )

        self.last_mapping_mode_flag = 0.0
        mapped = palm_xyz[self.palm_to_robot_axis_map]
        return self.palm_to_robot_axis_sign * mapped

    def normalize_axis(self, value, lower, upper):
        denom = max(float(upper - lower), 1e-9)
        return float(np.clip((float(value) - lower) / denom, 0.0, 1.0))

    def apply_relative_deadband(self, delta_xyz):
        return np.where(
            np.abs(delta_xyz) < self.relative_deadband_m,
            0.0,
            delta_xyz,
        )

    def apply_hmd_deadband(self, delta_xyz):
        return np.where(
            np.abs(delta_xyz) < self.hmd_deadband_m,
            0.0,
            delta_xyz,
        )

    def make_target_raw_xyz(self, current_ee_xyz):
        palm_abs_robot = self.last_palm_abs_robot.copy()
        if self.last_palm_xyz is not None:
            palm_abs_robot = self.palm_xyz_to_robot_abs(self.last_palm_xyz)
        r = float(math.hypot(float(palm_abs_robot[0]), float(palm_abs_robot[1])))
        yaw = float(math.atan2(float(palm_abs_robot[1]), float(palm_abs_robot[0])))
        self.last_palm_delta_xyz = np.zeros(3, dtype=np.float64)
        self.last_hmd_delta_debug_xyz = np.zeros(3, dtype=np.float64)
        self.last_hmd_extend = 0.0
        self.last_hmd_lateral = 0.0
        self.last_hmd_depth = 0.0
        self.last_absolute_mapped_delta_robot = np.zeros(3, dtype=np.float64)

        if self.target_mapping_mode == "cylindrical":
            self.last_target_mapping_mode_flag = 0.0
            self.last_normalized_xyz = np.zeros(3, dtype=np.float64)
            r_target = float(np.clip(
                self.r_gain * r + self.r_offset,
                self.min_r,
                self.max_r))
            x_target = r_target * math.cos(yaw)
            y_target = float(np.clip(
                self.y_gain * r_target * math.sin(yaw) + self.y_offset,
                self.min_y,
                self.max_y))
            z_target = float(np.clip(
                self.z_gain * float(palm_abs_robot[2]) + self.z_offset,
                self.min_z,
                self.max_z))
        elif self.target_mapping_mode == "direct_xyz":
            self.last_target_mapping_mode_flag = 1.0
            self.last_normalized_xyz = np.zeros(3, dtype=np.float64)
            x_target = float(np.clip(
                self.x_gain * float(palm_abs_robot[0]) + self.x_offset,
                self.min_x,
                self.max_x))
            y_target = float(np.clip(
                self.y_gain * float(palm_abs_robot[1]) + self.y_offset,
                self.min_y,
                self.max_y))
            z_target = float(np.clip(
                self.z_gain * float(palm_abs_robot[2]) + self.z_offset,
                self.min_z,
                self.max_z))
        elif self.target_mapping_mode == "linear_xyz":
            self.last_target_mapping_mode_flag = 2.0
            normalized_xyz = np.array([
                self.normalize_axis(palm_abs_robot[0], self.palm_x_min, self.palm_x_max),
                self.normalize_axis(palm_abs_robot[1], self.palm_y_min, self.palm_y_max),
                self.normalize_axis(palm_abs_robot[2], self.palm_z_min, self.palm_z_max),
            ], dtype=np.float64)
            self.last_normalized_xyz = normalized_xyz.copy()
            x_target = self.target_x_min + normalized_xyz[0] * (
                self.target_x_max - self.target_x_min)
            y_target = self.target_y_min + normalized_xyz[1] * (
                self.target_y_max - self.target_y_min)
            z_target = self.target_z_min + normalized_xyz[2] * (
                self.target_z_max - self.target_z_min)
        elif self.target_mapping_mode == "relative_ee":
            self.last_target_mapping_mode_flag = 3.0
            self.last_normalized_xyz = np.zeros(3, dtype=np.float64)
            if (self.force_relative_anchor_reset or
                    self.palm_anchor_xyz is None or self.ee_anchor_xyz is None):
                self.palm_anchor_xyz = palm_abs_robot.copy()
                self.ee_anchor_xyz = current_ee_xyz.copy()
                self.force_relative_anchor_reset = False
                self.get_logger().info(
                    "Relative anchor reset "
                    f"palm_anchor={np.round(self.palm_anchor_xyz, 4).tolist()} "
                    f"ee_anchor={np.round(self.ee_anchor_xyz, 4).tolist()}")

            palm_delta_xyz = self.apply_relative_deadband(
                palm_abs_robot - self.palm_anchor_xyz)
            self.last_palm_delta_xyz = palm_delta_xyz.copy()
            target_xyz = self.ee_anchor_xyz + self.relative_scale_xyz * palm_delta_xyz
            target_xyz = clamp_array(
                target_xyz,
                np.array([self.min_x, self.min_y, self.min_z], dtype=np.float64),
                np.array([self.max_x, self.max_y, self.max_z], dtype=np.float64),
            )
            x_target = float(target_xyz[0])
            y_target = float(target_xyz[1])
            z_target = float(target_xyz[2])
        elif self.target_mapping_mode == "hmd_relative_ee":
            self.last_target_mapping_mode_flag = 4.0
            self.last_normalized_xyz = np.zeros(3, dtype=np.float64)
            if self.force_relative_anchor_reset or self.ee_anchor_xyz is None:
                self.ee_anchor_xyz = current_ee_xyz.copy()
                self.force_relative_anchor_reset = False
                self.get_logger().info(
                    "Relative anchor reset "
                    f"ee_anchor={np.round(self.ee_anchor_xyz, 4).tolist()}")

            if self.last_hmd_delta_xyz is None:
                hmd_delta_xyz = np.zeros(3, dtype=np.float64)
            else:
                hmd_delta_xyz = self.apply_hmd_deadband(self.last_hmd_delta_xyz)
            self.last_hmd_delta_debug_xyz = hmd_delta_xyz.copy()

            self.last_hmd_lateral = -float(hmd_delta_xyz[0])
            self.last_hmd_depth = float(hmd_delta_xyz[1])
            self.last_hmd_extend = float(hmd_delta_xyz[2])
            target_xyz = np.array([
                self.ee_anchor_xyz[0] + self.hmd_depth_scale * self.last_hmd_depth,
                self.ee_anchor_xyz[1] + self.hmd_lateral_scale * self.last_hmd_lateral,
                self.ee_anchor_xyz[2] + self.hmd_extend_scale * self.last_hmd_extend,
            ], dtype=np.float64)
            target_xyz = clamp_array(
                target_xyz,
                np.array([self.min_x, self.min_y, self.min_z], dtype=np.float64),
                np.array([self.max_x, self.max_y, self.max_z], dtype=np.float64),
            )
            x_target = float(target_xyz[0])
            y_target = float(target_xyz[1])
            z_target = float(target_xyz[2])
        else:
            self.last_target_mapping_mode_flag = 5.0
            self.last_normalized_xyz = np.zeros(3, dtype=np.float64)
            if self.latest_palm_world_mr is None:
                now = seconds_now(self)
                if now - self.last_absolute_wait_warn_time >= 1.0:
                    self.last_absolute_wait_warn_time = now
                    self.get_logger().warn(
                        "absolute_scaled_ee waiting for /palm_pose_world")
                if self.last_target_xyz is not None:
                    target_xyz = self.last_target_xyz.copy()
                else:
                    target_xyz = current_ee_xyz.copy()
            else:
                palm_world_mr = self.latest_palm_world_mr.copy()
                self.last_palm_world_mr = palm_world_mr.copy()
                mapped_delta_robot = (
                    self.absolute_r_robot_from_mr @ (palm_world_mr - self.absolute_mr_center)
                )
                self.last_absolute_mapped_delta_robot = mapped_delta_robot.copy()
                target_xyz = (
                    self.absolute_robot_center +
                    self.absolute_scale_xyz * mapped_delta_robot
                )
                target_xyz = clamp_array(
                    target_xyz,
                    np.array([self.min_x, self.min_y, self.min_z], dtype=np.float64),
                    np.array([self.max_x, self.max_y, self.max_z], dtype=np.float64),
                )
            x_target = float(target_xyz[0])
            y_target = float(target_xyz[1])
            z_target = float(target_xyz[2])

        target_xyz = np.array([x_target, y_target, z_target], dtype=np.float64)

        self.last_palm_abs_robot = palm_abs_robot.copy()
        if self.palm_anchor_xyz is not None:
            self.last_palm_anchor_xyz = self.palm_anchor_xyz.copy()
        if self.ee_anchor_xyz is not None:
            self.last_ee_anchor_xyz = self.ee_anchor_xyz.copy()
        self.last_r = r
        self.last_yaw = yaw
        self.last_target_raw_xyz = target_xyz.copy()
        self.last_target_guarded_xyz = target_xyz.copy()
        self.last_target_step_limited_xyz = target_xyz.copy()
        return target_xyz

    def limit_target_step(self, target_xyz):
        limited_target = target_xyz.copy()
        if self.last_target_xyz is not None:
            delta = limited_target - self.last_target_xyz
            delta_norm = float(np.linalg.norm(delta))
            if self.max_target_step_m > 0.0 and delta_norm > self.max_target_step_m:
                limited_target = self.last_target_xyz + delta * (
                    self.max_target_step_m / delta_norm)

        self.last_target_xyz = limited_target.copy()
        self.last_target_step_limited_xyz = limited_target.copy()
        return limited_target

    def solve_ik_for_target(self, target_xyz, q_current):
        q_solution, _final_error = dls_ik(
            target_xyz,
            q_current,
            self.ik_lambda,
            self.ik_step_scale,
            self.ik_iters)
        q_solution = clamp_array(q_solution, JOINT_LIMITS[:, 0], JOINT_LIMITS[:, 1])
        fk_after = fk_xyz(q_solution)
        error_after = float(np.linalg.norm(target_xyz - fk_after))
        return q_solution, fk_after, error_after

    def target_is_reachable(self, q_solution, error_after):
        joint2_hit = float(q_solution[1]) > self.guard_joint2_max
        error_hit = float(error_after) > self.guard_error_max
        return not joint2_hit and not error_hit, joint2_hit, error_hit

    def apply_reachability_guard(self, target_raw_xyz, q_current, now):
        self.last_reachability_guard_enabled = (
            1.0 if self.enable_reachability_guard and
            self.target_mapping_mode in (
                "linear_xyz", "relative_ee", "absolute_scaled_ee") else 0.0
        )
        self.last_selected_guard_scale = 1.0
        self.last_guard_joint2_hit = 0.0
        self.last_guard_error_hit = 0.0

        if not self.enable_reachability_guard:
            self.last_target_guarded_xyz = target_raw_xyz.copy()
            fk_current = fk_xyz(q_current)
            error_current = float(np.linalg.norm(target_raw_xyz - fk_current))
            return target_raw_xyz.copy(), q_current.copy(), fk_current, error_current

        if self.last_reachability_guard_enabled <= 0.0:
            q_solution, fk_after, error_after = self.solve_ik_for_target(
                target_raw_xyz, q_current)
            self.last_target_guarded_xyz = target_raw_xyz.copy()
            reachable, joint2_hit, error_hit = self.target_is_reachable(q_solution, error_after)
            del reachable
            self.last_guard_joint2_hit = 1.0 if joint2_hit else 0.0
            self.last_guard_error_hit = 1.0 if error_hit else 0.0
            return target_raw_xyz.copy(), q_solution, fk_after, error_after

        best = None
        guard_min_x = self.target_x_min
        if self.target_mapping_mode in ("relative_ee", "absolute_scaled_ee"):
            guard_min_x = self.min_x
        for scale in self.guard_scale_candidates:
            scale = float(scale)
            candidate = np.array([
                guard_min_x + scale * (
                    target_raw_xyz[0] - guard_min_x),
                target_raw_xyz[1],
                target_raw_xyz[2],
            ], dtype=np.float64)
            q_solution, fk_after, error_after = self.solve_ik_for_target(candidate, q_current)
            reachable, joint2_hit, error_hit = self.target_is_reachable(q_solution, error_after)
            if best is None or error_after < best[3]:
                best = (candidate, q_solution, fk_after, error_after, scale, joint2_hit, error_hit)
            if reachable:
                self.last_selected_guard_scale = scale
                self.last_guard_joint2_hit = 0.0
                self.last_guard_error_hit = 0.0
                self.last_target_guarded_xyz = candidate.copy()
                return candidate, q_solution, fk_after, error_after

        candidate, q_solution, fk_after, error_after, scale, joint2_hit, error_hit = best
        self.last_selected_guard_scale = scale
        self.last_guard_joint2_hit = 1.0 if joint2_hit else 0.0
        self.last_guard_error_hit = 1.0 if error_hit else 0.0
        self.last_target_guarded_xyz = candidate.copy()
        if now - self.last_guard_warn_time >= 1.0:
            self.last_guard_warn_time = now
            self.get_logger().warn(
                "reachability_guard could not find a fully reachable target; "
                f"using best_error candidate scale={scale:.3f} "
                f"target_guarded={np.round(candidate, 4).tolist()} "
                f"q2={q_solution[1]:.4f} error_after={error_after:.5f}")
        return candidate, q_solution, fk_after, error_after

    def publish_trajectory(self, joints_rad):
        if self.mode not in ("gazebo", "real") or self.trajectory_pub is None:
            return

        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = [float(v) for v in joints_rad]
        point.time_from_start = duration_from_seconds(self.trajectory_time_from_start_sec)
        msg.points = [point]
        self.trajectory_pub.publish(msg)

    def publish_relative_visualization(self, target_xyz):
        if self.palm_anchor_xyz is None or self.ee_anchor_xyz is None:
            return

        self.palm_anchor_robot_pub.publish(self.make_pose_stamped(self.palm_anchor_xyz))
        self.ee_anchor_robot_pub.publish(self.make_pose_stamped(self.ee_anchor_xyz))
        self.palm_delta_robot_pub.publish(self.make_vector3_stamped(self.last_palm_delta_xyz))
        self.target_pose_robot_pub.publish(self.make_pose_stamped(target_xyz))

    def publish_debug(self, target_xyz):
        self.publish_float_array(self.target_pub, target_xyz)
        self.publish_float_array(self.current_ee_pub, self.last_fk_before)
        self.publish_float_array(self.palm_abs_pub, self.last_palm_abs_robot)
        self.publish_relative_visualization(target_xyz)
        self.publish_float_array(self.ik_debug_pub, [
            self.last_error_before,
            self.last_error_after,
            target_xyz[0], target_xyz[1], target_xyz[2],
            self.last_fk_before[0], self.last_fk_before[1], self.last_fk_before[2],
            self.last_fk_after[0], self.last_fk_after[1], self.last_fk_after[2],
            self.last_palm_abs_robot[0],
            self.last_palm_abs_robot[1],
            self.last_palm_abs_robot[2],
            self.last_r,
            self.last_yaw,
            self.last_q_current[0],
            self.last_q_current[1],
            self.last_q_current[2],
            self.last_q_current[3],
            self.last_q_current[4],
            self.last_q_solution[0],
            self.last_q_solution[1],
            self.last_q_solution[2],
            self.last_q_solution[3],
            self.last_q_solution[4],
            self.last_mapping_mode_flag,
            self.last_target_mapping_mode_flag,
            self.r_robot_from_mr[0, 0],
            self.r_robot_from_mr[0, 1],
            self.r_robot_from_mr[0, 2],
            self.r_robot_from_mr[1, 0],
            self.r_robot_from_mr[1, 1],
            self.r_robot_from_mr[1, 2],
            self.r_robot_from_mr[2, 0],
            self.r_robot_from_mr[2, 1],
            self.r_robot_from_mr[2, 2],
            self.t_robot_from_mr[0],
            self.t_robot_from_mr[1],
            self.t_robot_from_mr[2],
            self.last_palm_abs_robot[0],
            self.last_palm_abs_robot[1],
            self.last_palm_abs_robot[2],
            self.last_normalized_xyz[0],
            self.last_normalized_xyz[1],
            self.last_normalized_xyz[2],
            self.last_target_raw_xyz[0],
            self.last_target_raw_xyz[1],
            self.last_target_raw_xyz[2],
            self.last_target_guarded_xyz[0],
            self.last_target_guarded_xyz[1],
            self.last_target_guarded_xyz[2],
            self.last_target_step_limited_xyz[0],
            self.last_target_step_limited_xyz[1],
            self.last_target_step_limited_xyz[2],
            self.last_reachability_guard_enabled,
            self.last_selected_guard_scale,
            self.last_guard_joint2_hit,
            self.last_guard_error_hit,
            self.last_palm_anchor_xyz[0],
            self.last_palm_anchor_xyz[1],
            self.last_palm_anchor_xyz[2],
            self.last_ee_anchor_xyz[0],
            self.last_ee_anchor_xyz[1],
            self.last_ee_anchor_xyz[2],
            self.last_palm_delta_xyz[0],
            self.last_palm_delta_xyz[1],
            self.last_palm_delta_xyz[2],
            self.relative_scale_xyz[0],
            self.relative_scale_xyz[1],
            self.relative_scale_xyz[2],
            self.last_hmd_delta_debug_xyz[0],
            self.last_hmd_delta_debug_xyz[1],
            self.last_hmd_delta_debug_xyz[2],
            self.last_hmd_extend,
            self.last_hmd_lateral,
            self.last_hmd_depth,
            self.hmd_extend_scale,
            self.hmd_lateral_scale,
            self.hmd_depth_scale,
            self.last_q_solution[0],
            self.last_q_solution[1],
            self.last_q_solution[2],
            self.last_q_solution[3],
            self.last_q_solution[4],
            self.last_error_after,
            self.last_palm_world_mr[0],
            self.last_palm_world_mr[1],
            self.last_palm_world_mr[2],
            self.absolute_mr_center[0],
            self.absolute_mr_center[1],
            self.absolute_mr_center[2],
            self.absolute_robot_center[0],
            self.absolute_robot_center[1],
            self.absolute_robot_center[2],
            self.last_absolute_mapped_delta_robot[0],
            self.last_absolute_mapped_delta_robot[1],
            self.last_absolute_mapped_delta_robot[2],
            self.absolute_scale_xyz[0],
            self.absolute_scale_xyz[1],
            self.absolute_scale_xyz[2],
            self.reset_count,
            self.last_reset_accepted,
        ])

    def log_debug_state(self, now, target_xyz):
        if now - self.last_debug_log_time < 1.0:
            return
        self.last_debug_log_time = now
        self.get_logger().info(
            "amir_abs_ik_debug "
            f"palm_mapping_mode={self.palm_mapping_mode} "
            f"target_mapping_mode={self.target_mapping_mode} "
            f"target_mapping_mode_flag={self.last_target_mapping_mode_flag:.0f} "
            f"palm_abs_robot={np.round(self.last_palm_abs_robot, 4).tolist()} "
            f"palm_anchor_xyz={np.round(self.last_palm_anchor_xyz, 4).tolist()} "
            f"ee_anchor_xyz={np.round(self.last_ee_anchor_xyz, 4).tolist()} "
            f"palm_delta_xyz={np.round(self.last_palm_delta_xyz, 4).tolist()} "
            f"relative_scale_xyz={np.round(self.relative_scale_xyz, 4).tolist()} "
            f"hmd_delta_xyz={np.round(self.last_hmd_delta_debug_xyz, 4).tolist()} "
            f"hmd_extend={self.last_hmd_extend:.4f} "
            f"hmd_lateral={self.last_hmd_lateral:.4f} "
            f"hmd_depth={self.last_hmd_depth:.4f} "
            f"hmd_extend_scale={self.hmd_extend_scale:.4f} "
            f"hmd_lateral_scale={self.hmd_lateral_scale:.4f} "
            f"hmd_depth_scale={self.hmd_depth_scale:.4f} "
            f"palm_world_mr={np.round(self.last_palm_world_mr, 4).tolist()} "
            f"absolute_mr_center={np.round(self.absolute_mr_center, 4).tolist()} "
            f"absolute_robot_center={np.round(self.absolute_robot_center, 4).tolist()} "
            f"mapped_delta_robot={np.round(self.last_absolute_mapped_delta_robot, 4).tolist()} "
            f"absolute_scale_xyz={np.round(self.absolute_scale_xyz, 4).tolist()} "
            f"relative_viz_published={int(self.palm_anchor_xyz is not None and self.ee_anchor_xyz is not None)} "
            f"published_palm_anchor_robot={np.round(self.last_palm_anchor_xyz, 4).tolist()} "
            f"published_ee_anchor_robot={np.round(self.last_ee_anchor_xyz, 4).tolist()} "
            f"published_target_pose_robot={np.round(self.last_target_step_limited_xyz, 4).tolist()} "
            f"R_robot_from_mr={np.round(self.r_robot_from_mr, 4).tolist()} "
            f"t_robot_from_mr={np.round(self.t_robot_from_mr, 4).tolist()} "
            f"p_robot={np.round(self.last_palm_abs_robot, 4).tolist()} "
            f"normalized_xyz={np.round(self.last_normalized_xyz, 4).tolist()} "
            f"target_raw_xyz={np.round(self.last_target_raw_xyz, 4).tolist()} "
            f"target_guarded_xyz={np.round(self.last_target_guarded_xyz, 4).tolist()} "
            f"target_step_limited_xyz={np.round(self.last_target_step_limited_xyz, 4).tolist()} "
            f"reachability_guard_enabled={self.last_reachability_guard_enabled:.0f} "
            f"selected_guard_scale={self.last_selected_guard_scale:.3f} "
            f"guard_joint2_hit={self.last_guard_joint2_hit:.0f} "
            f"guard_error_hit={self.last_guard_error_hit:.0f} "
            f"r={self.last_r:.4f} "
            f"yaw={self.last_yaw:.4f} "
            f"target_xyz={np.round(target_xyz, 4).tolist()} "
            f"x_gain={self.x_gain:.4f} "
            f"x_offset={self.x_offset:.4f} "
            f"y_gain={self.y_gain:.4f} "
            f"y_offset={self.y_offset:.4f} "
            f"z_gain={self.z_gain:.4f} "
            f"z_offset={self.z_offset:.4f} "
            f"fk_before={np.round(self.last_fk_before, 4).tolist()} "
            f"fk_after={np.round(self.last_fk_after, 4).tolist()} "
            f"error_before={self.last_error_before:.5f} "
            f"error_after={self.last_error_after:.5f} "
            f"q_current={np.round(self.last_q_current, 4).tolist()} "
            f"q_solution={np.round(self.last_q_solution, 4).tolist()}")

    def control_loop(self):
        if (self.target_mapping_mode not in ("hmd_relative_ee", "absolute_scaled_ee") and
                self.last_palm_xyz is None):
            return

        now = seconds_now(self)
        q_current = self.current_joints.copy()
        fk_before = fk_xyz(q_current)
        target_raw_xyz = self.make_target_raw_xyz(fk_before)
        target_guarded_xyz, _guard_q, _guard_fk, _guard_error = self.apply_reachability_guard(
            target_raw_xyz, q_current, now)
        target_xyz = self.limit_target_step(target_guarded_xyz)
        error_before = float(np.linalg.norm(target_xyz - fk_before))

        q_solution, fk_after, error_after = self.solve_ik_for_target(target_xyz, q_current)

        self.last_q_current = q_current.copy()
        self.last_q_solution = q_solution.copy()
        self.last_fk_before = fk_before.copy()
        self.last_fk_after = fk_after.copy()
        self.last_error_before = error_before
        self.last_error_after = error_after

        self.publish_debug(target_xyz)
        self.publish_trajectory(q_solution)
        self.log_debug_state(now, target_xyz)


def main(args=None):
    rclpy.init(args=args)
    node = AmirAbsolutePalmIkNode()
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
