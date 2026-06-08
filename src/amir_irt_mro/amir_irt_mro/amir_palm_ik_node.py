#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Relative palm-pose IK controller for Amir."""

import math

import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseStamped, Vector3Stamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


JOINT_NAMES = ["Joint_1", "Joint_2", "Joint_3", "Joint_4", "Joint_5"]

L1_OFFSET = 0.248
LINK_LENGTHS = np.array([0.310, 0.310, 0.148], dtype=np.float64)

DEFAULT_JOINTS = np.array([0.0, 1.3535, -2.5596, 0.5270, 0.0], dtype=np.float64)

JOINT_LIMITS = np.array([
    [-2.96706, 2.96706],
    [0.0, 2.356194],
    [-2.792527, 0.0],
    [-2.094395, 1.308997],
    [-2.75762, 2.75762],
], dtype=np.float64)

URDF_SOURCE_FILE = (
    "/home/sota/humbleble_sim_ws/install/amir_description/share/"
    "amir_description/urdf/amir_mecanum3_sim.xacro"
)
URDF_DETAIL_SOURCE_FILE = (
    "/home/sota/humbleble_sim_ws/install/amir_description/share/"
    "amir_description/urdf/amir_for_rover.xacro"
)
URDF_BASE_LINK = "link0_1"
URDF_TIP_LINK = "gripper_base_1"
URDF_JOINT_PARENTS = ["link0_1", "link1_1", "link2_1", "link3_1", "link4_1"]
URDF_JOINT_CHILDREN = ["link1_1", "link2_1", "link3_1", "link4_1", "gripper_base_1"]
URDF_JOINT_ORIGINS_XYZ = np.array([
    [-0.0, -0.001523, -0.097],
    [-0.0, 0.0, 0.097],
    [0.0, 0.31, -0.0],
    [-0.0, 0.31, 0.0],
    [0.0, 0.0405, 0.0],
], dtype=np.float64)
URDF_JOINT_ORIGINS_RPY = np.zeros((5, 3), dtype=np.float64)
URDF_JOINT_AXES = np.array([
    [0.0, 0.0, 1.0],
    [1.0, 0.0, 0.0],
    [1.0, 0.0, 0.0],
    [1.0, 0.0, 0.0],
    [0.0, 1.0, 0.0],
], dtype=np.float64)


def rpy_to_rot(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]], dtype=np.float64)
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]], dtype=np.float64)
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)
    return rz @ ry @ rx


def axis_angle_to_rot(axis, angle):
    axis = np.array(axis, dtype=np.float64)
    norm = float(np.linalg.norm(axis))
    if norm <= 1e-12:
        return np.eye(3, dtype=np.float64)
    x, y, z = axis / norm
    c = math.cos(angle)
    s = math.sin(angle)
    one_c = 1.0 - c
    return np.array([
        [c + x * x * one_c, x * y * one_c - z * s, x * z * one_c + y * s],
        [y * x * one_c + z * s, c + y * y * one_c, y * z * one_c - x * s],
        [z * x * one_c - y * s, z * y * one_c + x * s, c + z * z * one_c],
    ], dtype=np.float64)


def make_transform(xyz, rpy):
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = rpy_to_rot(float(rpy[0]), float(rpy[1]), float(rpy[2]))
    transform[:3, 3] = np.array(xyz, dtype=np.float64)
    return transform


def transform_about_axis(axis, q):
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = axis_angle_to_rot(axis, float(q))
    return transform


def clamp_array(value, lower, upper):
    return np.minimum(np.maximum(value, lower), upper)


def seconds_now(node):
    return node.get_clock().now().nanoseconds * 1e-9


def as_bool(value):
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def map_range_clamped(v, in_min, in_max, out_min, out_max):
    if abs(in_max - in_min) < 1e-9:
        return 0.5 * (out_min + out_max)
    t = (v - in_min) / (in_max - in_min)
    t = max(0.0, min(1.0, t))
    return out_min + t * (out_max - out_min)


def duration_from_seconds(value):
    sec = int(math.floor(max(value, 0.0)))
    nanosec = int(round((max(value, 0.0) - sec) * 1_000_000_000))
    if nanosec >= 1_000_000_000:
        sec += 1
        nanosec -= 1_000_000_000
    return Duration(sec=sec, nanosec=nanosec)


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


def fk_xyz_urdf_like(joints):
    transform = np.eye(4, dtype=np.float64)
    for idx, q in enumerate(joints):
        transform = (
            transform @
            make_transform(URDF_JOINT_ORIGINS_XYZ[idx], URDF_JOINT_ORIGINS_RPY[idx]) @
            transform_about_axis(URDF_JOINT_AXES[idx], q)
        )
    return transform[:3, 3].copy()


def numeric_jacobian(joints, fk_func=fk_xyz, eps=1e-5):
    base = fk_func(joints)
    jac = np.zeros((3, 5), dtype=np.float64)
    for idx in range(5):
        shifted = joints.copy()
        shifted[idx] += eps
        jac[:, idx] = (fk_func(shifted) - base) / eps
    return jac


def lm_ik(target_xyz, seed_joints, max_iters=60, fk_func=fk_xyz):
    joints = seed_joints.copy()
    damping = 2e-3

    for _ in range(max_iters):
        current = fk_func(joints)
        error = target_xyz - current
        if float(np.linalg.norm(error)) < 1e-4:
            break

        jac = numeric_jacobian(joints, fk_func=fk_func)
        lhs = jac.T @ jac + damping * np.eye(5)
        rhs = jac.T @ error
        try:
            delta = np.linalg.solve(lhs, rhs)
        except np.linalg.LinAlgError:
            delta = np.linalg.pinv(lhs) @ rhs

        delta = np.clip(delta, -0.08, 0.08)
        joints = clamp_array(joints + delta, JOINT_LIMITS[:, 0], JOINT_LIMITS[:, 1])

    final_error = float(np.linalg.norm(target_xyz - fk_func(joints)))
    return joints, final_error


class AmirPalmIkNode(Node):
    def __init__(self):
        super().__init__("amir_palm_ik_node")

        self.declare_parameter("mode", "sim")
        self.declare_parameter("control_rate_hz", 30.0)
        self.declare_parameter("kinematics_model", "simple_3link")
        self.declare_parameter("enable_kinematics_target_adapter", True)
        self.declare_parameter(
            "kinematics_target_adapter_mode", "control_to_urdf_y_forward")
        self.declare_parameter("palm_pose_topic", "/palm_pose_world")
        self.declare_parameter(
            "control_center_topic", "/palm_pose_control_center_world")
        self.declare_parameter("use_control_center", True)
        self.declare_parameter("require_control_center", True)
        self.declare_parameter("target_mapping_mode", "relative")
        self.declare_parameter("palm_motion_gain", 1.0)
        self.declare_parameter("scale_x", 0.6)
        self.declare_parameter("scale_y", 0.3)
        self.declare_parameter("scale_z", 0.6)
        self.declare_parameter("deadzone_m", 0.005)
        self.declare_parameter("max_delta_norm_m", 0.30)
        self.declare_parameter("max_target_step_m", 0.003)
        self.declare_parameter("lowpass_alpha", 0.25)
        self.declare_parameter("input_timeout_sec", 0.5)
        self.declare_parameter("target_timeout_sec", 0.5)
        self.declare_parameter("workspace_min", [0.12, -0.25, 0.05])
        self.declare_parameter("workspace_max", [0.45, 0.25, 0.45])
        self.declare_parameter("workspace_x_min", 0.12)
        self.declare_parameter("workspace_x_max", 0.45)
        self.declare_parameter("workspace_y_min", -0.25)
        self.declare_parameter("workspace_y_max", 0.25)
        self.declare_parameter("workspace_z_min", 0.05)
        self.declare_parameter("workspace_z_max", 0.45)
        self.declare_parameter("enable_reach_radius_limit", True)
        self.declare_parameter("reach_radius_max_m", 0.85)
        self.declare_parameter("reach_radius_margin_m", 0.03)
        self.declare_parameter("reach_radius_origin_x", 0.0)
        self.declare_parameter("reach_radius_origin_y", 0.0)
        self.declare_parameter("reach_radius_origin_z", 0.0)
        self.declare_parameter("trajectory_time_from_start_sec", 0.2)
        self.declare_parameter("command_unit", "rad")
        self.declare_parameter("axis_map", [2, 0, 1])
        self.declare_parameter("axis_sign", [1.0, 1.0, 1.0])
        self.declare_parameter("palm_to_robot_axis_map", [2, 0, 1])
        self.declare_parameter("palm_to_robot_axis_sign", [1.0, 1.0, 1.0])
        self.declare_parameter("palm_axis_gain", [1.0, 1.0, 1.0])
        self.declare_parameter("mr_x_min", -0.30)
        self.declare_parameter("mr_x_max", 0.30)
        self.declare_parameter("mr_y_min", -0.50)
        self.declare_parameter("mr_y_max", 0.20)
        self.declare_parameter("mr_z_min", 0.00)
        self.declare_parameter("mr_z_max", 0.70)
        self.declare_parameter("abs_robot_x_min", 0.20)
        self.declare_parameter("abs_robot_x_max", 0.55)
        self.declare_parameter("abs_robot_y_min", -0.15)
        self.declare_parameter("abs_robot_y_max", 0.15)
        self.declare_parameter("abs_robot_z_min", 0.35)
        self.declare_parameter("abs_robot_z_max", 0.90)
        self.declare_parameter("debug_target_step_x", 0.0)
        self.declare_parameter("palm_target_mode", "relative")
        self.declare_parameter("cylindrical_r_gain", 1.0)
        self.declare_parameter("cylindrical_z_gain", 1.0)
        self.declare_parameter("cylindrical_y_gain", 0.0)
        self.declare_parameter("cylindrical_r_offset", 0.0)
        self.declare_parameter("cylindrical_z_offset", 0.0)
        self.declare_parameter("cylindrical_y_offset", 0.0)
        self.declare_parameter("cylindrical_min_r", 0.05)
        self.declare_parameter("cylindrical_max_r", 0.75)
        self.declare_parameter("cylindrical_min_z", 0.00)
        self.declare_parameter("cylindrical_max_z", 0.80)
        self.declare_parameter("enable_range_scaled_palm", False)
        self.declare_parameter("human_motion_range", [0.35, 0.30, 0.30])
        self.declare_parameter("robot_motion_range", [0.60, 0.40, 0.40])
        self.declare_parameter("range_scale_gain", 1.0)
        self.declare_parameter("range_scale_max", [3.0, 3.0, 3.0])
        self.declare_parameter("enable_extension_bias", False)
        self.declare_parameter("extension_bias_gain", 0.05)
        self.declare_parameter("extension_trigger_x", 0.02)
        self.declare_parameter("extension_trigger_norm", 0.02)
        self.declare_parameter("extension_use_motion_norm", True)
        self.declare_parameter("extension_posture", [0.0, 1.20, -2.20, -0.70, 0.0])
        self.declare_parameter("joint_limit_avoidance_gain", 0.02)
        self.declare_parameter("enable_reach_approach", False)
        self.declare_parameter("reach_trigger_error", 0.03)
        self.declare_parameter("reach_full_error", 0.15)
        self.declare_parameter("reach_bias_gain", 0.25)
        self.declare_parameter("reach_posture", [0.0, 2.10, -0.40, -0.15, 0.0])
        self.declare_parameter("reach_directional_yaw", True)
        self.declare_parameter("reach_yaw_trigger_norm", 0.01)

        self.mode = str(self.get_parameter("mode").value).strip().lower()
        if self.mode not in ("sim", "gazebo", "real"):
            self.get_logger().warn(f'Invalid mode "{self.mode}". Falling back to sim.')
            self.mode = "sim"

        self.command_unit = str(self.get_parameter("command_unit").value).strip().lower()
        if self.command_unit not in ("rad", "mrad"):
            self.get_logger().warn(
                f'Invalid command_unit "{self.command_unit}". Falling back to rad.')
            self.command_unit = "rad"

        self.control_rate_hz = max(float(self.get_parameter("control_rate_hz").value), 0.1)
        self.kinematics_model = str(
            self.get_parameter("kinematics_model").value).strip().lower()
        self.enable_kinematics_target_adapter = as_bool(
            self.get_parameter("enable_kinematics_target_adapter").value)
        self.kinematics_target_adapter_mode = str(
            self.get_parameter("kinematics_target_adapter_mode").value).strip().lower()
        self.palm_pose_topic = str(self.get_parameter("palm_pose_topic").value).strip()
        self.control_center_topic = str(
            self.get_parameter("control_center_topic").value).strip()
        if not self.palm_pose_topic.startswith("/"):
            self.get_logger().warn(
                f'palm_pose_topic "{self.palm_pose_topic}" should be absolute. '
                "Using /palm_pose_world.")
            self.palm_pose_topic = "/palm_pose_world"
        if not self.control_center_topic.startswith("/"):
            self.get_logger().warn(
                f'control_center_topic "{self.control_center_topic}" should be absolute. '
                "Using /palm_pose_control_center_world.")
            self.control_center_topic = "/palm_pose_control_center_world"
        self.use_control_center = as_bool(self.get_parameter("use_control_center").value)
        self.require_control_center = as_bool(
            self.get_parameter("require_control_center").value)
        self.target_mapping_mode = str(
            self.get_parameter("target_mapping_mode").value).strip().lower()
        self.absolute_axis_mapping = "unity_z_to_x_unity_x_to_y_unity_y_to_z"
        self.palm_motion_gain = float(self.get_parameter("palm_motion_gain").value)
        self.scale_xyz = np.array([
            float(self.get_parameter("scale_x").value),
            float(self.get_parameter("scale_y").value),
            float(self.get_parameter("scale_z").value),
        ], dtype=np.float64)
        self.deadzone_m = max(float(self.get_parameter("deadzone_m").value), 0.0)
        self.max_delta_norm_m = max(float(self.get_parameter("max_delta_norm_m").value), 0.0)
        self.max_target_step_m = max(float(self.get_parameter("max_target_step_m").value), 0.0)
        self.lowpass_alpha = float(np.clip(
            float(self.get_parameter("lowpass_alpha").value), 0.0, 1.0))
        self.input_timeout_sec = max(float(self.get_parameter("input_timeout_sec").value), 0.0)
        self.target_timeout_sec = max(
            float(self.get_parameter("target_timeout_sec").value), 0.0)
        workspace_min_param = np.array(
            self.get_parameter("workspace_min").value, dtype=np.float64)
        workspace_max_param = np.array(
            self.get_parameter("workspace_max").value, dtype=np.float64)
        self.workspace_min = np.array([
            float(self.get_parameter("workspace_x_min").value),
            float(self.get_parameter("workspace_y_min").value),
            float(self.get_parameter("workspace_z_min").value),
        ], dtype=np.float64)
        self.workspace_max = np.array([
            float(self.get_parameter("workspace_x_max").value),
            float(self.get_parameter("workspace_y_max").value),
            float(self.get_parameter("workspace_z_max").value),
        ], dtype=np.float64)
        if (np.allclose(self.workspace_min, [0.12, -0.25, 0.05]) and
                np.allclose(self.workspace_max, [0.45, 0.25, 0.45]) and
                workspace_min_param.shape == (3,) and workspace_max_param.shape == (3,) and
                (not np.allclose(workspace_min_param, [0.12, -0.25, 0.05]) or
                 not np.allclose(workspace_max_param, [0.45, 0.25, 0.45]))):
            self.workspace_min = workspace_min_param
            self.workspace_max = workspace_max_param
        self.enable_reach_radius_limit = as_bool(
            self.get_parameter("enable_reach_radius_limit").value)
        self.reach_radius_max_m = max(
            float(self.get_parameter("reach_radius_max_m").value), 0.0)
        self.reach_radius_margin_m = max(
            float(self.get_parameter("reach_radius_margin_m").value), 0.0)
        self.reach_radius_origin = np.array([
            float(self.get_parameter("reach_radius_origin_x").value),
            float(self.get_parameter("reach_radius_origin_y").value),
            float(self.get_parameter("reach_radius_origin_z").value),
        ], dtype=np.float64)
        self.effective_reach_radius = max(
            self.reach_radius_max_m - self.reach_radius_margin_m, 0.0)
        self.trajectory_time_from_start_sec = float(
            self.get_parameter("trajectory_time_from_start_sec").value)
        axis_map = np.array(self.get_parameter("axis_map").value, dtype=np.int64)
        axis_sign = np.array(self.get_parameter("axis_sign").value, dtype=np.float64)
        legacy_axis_map = np.array(
            self.get_parameter("palm_to_robot_axis_map").value, dtype=np.int64)
        legacy_axis_sign = np.array(
            self.get_parameter("palm_to_robot_axis_sign").value, dtype=np.float64)
        if (np.array_equal(axis_map, np.array([2, 0, 1], dtype=np.int64)) and
                legacy_axis_map.shape == (3,) and
                not np.array_equal(legacy_axis_map, np.array([2, 0, 1], dtype=np.int64))):
            axis_map = legacy_axis_map
        if (axis_sign.shape == (3,) and legacy_axis_sign.shape == (3,) and
                np.allclose(axis_sign, [1.0, 1.0, 1.0]) and
                not np.allclose(legacy_axis_sign, [1.0, 1.0, 1.0])):
            axis_sign = legacy_axis_sign
        self.palm_to_robot_axis_map = axis_map
        self.palm_to_robot_axis_sign = axis_sign
        self.palm_axis_gain = np.array(
            self.get_parameter("palm_axis_gain").value, dtype=np.float64)
        self.mr_min = np.array([
            float(self.get_parameter("mr_x_min").value),
            float(self.get_parameter("mr_y_min").value),
            float(self.get_parameter("mr_z_min").value),
        ], dtype=np.float64)
        self.mr_max = np.array([
            float(self.get_parameter("mr_x_max").value),
            float(self.get_parameter("mr_y_max").value),
            float(self.get_parameter("mr_z_max").value),
        ], dtype=np.float64)
        self.abs_robot_min = np.array([
            float(self.get_parameter("abs_robot_x_min").value),
            float(self.get_parameter("abs_robot_y_min").value),
            float(self.get_parameter("abs_robot_z_min").value),
        ], dtype=np.float64)
        self.abs_robot_max = np.array([
            float(self.get_parameter("abs_robot_x_max").value),
            float(self.get_parameter("abs_robot_y_max").value),
            float(self.get_parameter("abs_robot_z_max").value),
        ], dtype=np.float64)
        self.debug_target_step_x = float(self.get_parameter("debug_target_step_x").value)
        self.palm_target_mode = str(self.get_parameter("palm_target_mode").value).strip().lower()
        self.cylindrical_r_gain = float(self.get_parameter("cylindrical_r_gain").value)
        self.cylindrical_z_gain = float(self.get_parameter("cylindrical_z_gain").value)
        self.cylindrical_y_gain = float(self.get_parameter("cylindrical_y_gain").value)
        self.cylindrical_r_offset = float(self.get_parameter("cylindrical_r_offset").value)
        self.cylindrical_z_offset = float(self.get_parameter("cylindrical_z_offset").value)
        self.cylindrical_y_offset = float(self.get_parameter("cylindrical_y_offset").value)
        self.cylindrical_min_r = float(self.get_parameter("cylindrical_min_r").value)
        self.cylindrical_max_r = float(self.get_parameter("cylindrical_max_r").value)
        self.cylindrical_min_z = float(self.get_parameter("cylindrical_min_z").value)
        self.cylindrical_max_z = float(self.get_parameter("cylindrical_max_z").value)
        self.enable_range_scaled_palm = as_bool(
            self.get_parameter("enable_range_scaled_palm").value)
        self.human_motion_range = np.array(
            self.get_parameter("human_motion_range").value, dtype=np.float64)
        self.robot_motion_range = np.array(
            self.get_parameter("robot_motion_range").value, dtype=np.float64)
        self.range_scale_gain = float(self.get_parameter("range_scale_gain").value)
        self.range_scale_max = np.array(
            self.get_parameter("range_scale_max").value, dtype=np.float64)
        self.enable_extension_bias = as_bool(self.get_parameter("enable_extension_bias").value)
        self.extension_bias_gain = max(
            float(self.get_parameter("extension_bias_gain").value), 0.0)
        self.extension_trigger_x = max(
            float(self.get_parameter("extension_trigger_x").value), 0.0)
        self.extension_trigger_norm = max(
            float(self.get_parameter("extension_trigger_norm").value), 0.0)
        self.extension_use_motion_norm = as_bool(
            self.get_parameter("extension_use_motion_norm").value)
        self.extension_posture = np.array(
            self.get_parameter("extension_posture").value, dtype=np.float64)
        self.joint_limit_avoidance_gain = max(
            float(self.get_parameter("joint_limit_avoidance_gain").value), 0.0)
        self.enable_reach_approach = as_bool(
            self.get_parameter("enable_reach_approach").value)
        self.reach_trigger_error = max(
            float(self.get_parameter("reach_trigger_error").value), 0.0)
        self.reach_full_error = max(
            float(self.get_parameter("reach_full_error").value), self.reach_trigger_error)
        self.reach_bias_gain = max(float(self.get_parameter("reach_bias_gain").value), 0.0)
        self.reach_posture = np.array(
            self.get_parameter("reach_posture").value, dtype=np.float64)
        self.reach_directional_yaw = as_bool(
            self.get_parameter("reach_directional_yaw").value)
        self.reach_yaw_trigger_norm = max(
            float(self.get_parameter("reach_yaw_trigger_norm").value), 0.0)

        if self.workspace_min.shape != (3,) or self.workspace_max.shape != (3,):
            self.get_logger().warn("workspace_min/max must be length 3. Using defaults.")
            self.workspace_min = np.array([0.12, -0.25, 0.05], dtype=np.float64)
            self.workspace_max = np.array([0.45, 0.25, 0.45], dtype=np.float64)
        if np.any(self.workspace_min > self.workspace_max):
            self.get_logger().warn("workspace_min is greater than workspace_max. Swapping bounds.")
            low = np.minimum(self.workspace_min, self.workspace_max)
            high = np.maximum(self.workspace_min, self.workspace_max)
            self.workspace_min = low
            self.workspace_max = high
        if self.kinematics_model not in ("simple_3link", "urdf_like"):
            self.get_logger().warn(
                f'Invalid kinematics_model "{self.kinematics_model}". Using simple_3link.')
            self.kinematics_model = "simple_3link"
        if self.kinematics_target_adapter_mode not in (
                "none", "control_to_urdf_y_forward"):
            self.get_logger().warn(
                f'Invalid kinematics_target_adapter_mode '
                f'"{self.kinematics_target_adapter_mode}". Using control_to_urdf_y_forward.')
            self.kinematics_target_adapter_mode = "control_to_urdf_y_forward"
        if self.target_mapping_mode not in ("relative", "absolute"):
            self.get_logger().warn(
                f'Invalid target_mapping_mode "{self.target_mapping_mode}". Using relative.')
            self.target_mapping_mode = "relative"
        if np.any(self.mr_min > self.mr_max):
            self.get_logger().warn("MR min range is greater than max. Swapping bounds.")
            low = np.minimum(self.mr_min, self.mr_max)
            high = np.maximum(self.mr_min, self.mr_max)
            self.mr_min = low
            self.mr_max = high
        if np.any(self.abs_robot_min > self.abs_robot_max):
            self.get_logger().warn(
                "Absolute robot min range is greater than max. Swapping bounds.")
            low = np.minimum(self.abs_robot_min, self.abs_robot_max)
            high = np.maximum(self.abs_robot_min, self.abs_robot_max)
            self.abs_robot_min = low
            self.abs_robot_max = high
        if (self.palm_to_robot_axis_map.shape != (3,) or
                sorted([int(v) for v in self.palm_to_robot_axis_map.tolist()]) != [0, 1, 2]):
            self.get_logger().warn(
                "axis_map must be a permutation of [0, 1, 2]. Using [2, 0, 1].")
            self.palm_to_robot_axis_map = np.array([2, 0, 1], dtype=np.int64)
        if self.palm_to_robot_axis_sign.shape != (3,):
            self.get_logger().warn("axis_sign must be length 3. Using [1.0, 1.0, 1.0].")
            self.palm_to_robot_axis_sign = np.array([1.0, 1.0, 1.0], dtype=np.float64)
        if not np.allclose(np.abs(self.palm_to_robot_axis_sign), [1.0, 1.0, 1.0]):
            self.get_logger().warn(
                "axis_sign is treated as signs only; normalizing values to +/-1.")
        self.palm_to_robot_axis_sign = np.where(
            self.palm_to_robot_axis_sign < 0.0, -1.0, 1.0).astype(np.float64)
        if self.palm_axis_gain.shape != (3,):
            self.get_logger().warn("palm_axis_gain must be length 3. Using [1.0, 1.0, 1.0].")
            self.palm_axis_gain = np.array([1.0, 1.0, 1.0], dtype=np.float64)
        if self.palm_target_mode not in ("relative", "cylindrical_absolute"):
            self.get_logger().warn(
                f'Invalid palm_target_mode "{self.palm_target_mode}". Using relative.')
            self.palm_target_mode = "relative"
        if self.cylindrical_min_r > self.cylindrical_max_r:
            self.cylindrical_min_r, self.cylindrical_max_r = (
                self.cylindrical_max_r, self.cylindrical_min_r)
        if self.cylindrical_min_z > self.cylindrical_max_z:
            self.cylindrical_min_z, self.cylindrical_max_z = (
                self.cylindrical_max_z, self.cylindrical_min_z)
        if self.human_motion_range.shape != (3,):
            self.get_logger().warn("human_motion_range must be length 3. "
                                   "Using [0.35, 0.30, 0.30].")
            self.human_motion_range = np.array([0.35, 0.30, 0.30], dtype=np.float64)
        if self.robot_motion_range.shape != (3,):
            self.get_logger().warn("robot_motion_range must be length 3. "
                                   "Using [0.60, 0.40, 0.40].")
            self.robot_motion_range = np.array([0.60, 0.40, 0.40], dtype=np.float64)
        if self.range_scale_max.shape != (3,):
            self.get_logger().warn("range_scale_max must be length 3. Using [3.0, 3.0, 3.0].")
            self.range_scale_max = np.array([3.0, 3.0, 3.0], dtype=np.float64)
        self.human_motion_range = np.maximum(self.human_motion_range, 1e-6)
        self.robot_motion_range = np.maximum(self.robot_motion_range, 0.0)
        self.range_scale_max = np.maximum(self.range_scale_max, 0.0)
        if self.extension_posture.shape != (5,):
            self.get_logger().warn("extension_posture must be length 5. "
                                   "Using [0.0, 1.20, -2.20, -0.70, 0.0].")
            self.extension_posture = np.array([0.0, 1.20, -2.20, -0.70, 0.0],
                                             dtype=np.float64)
        self.extension_posture = clamp_array(
            self.extension_posture, JOINT_LIMITS[:, 0], JOINT_LIMITS[:, 1])
        if self.reach_posture.shape != (5,):
            self.get_logger().warn("reach_posture must be length 5. "
                                   "Using [0.0, 2.10, -0.40, -0.15, 0.0].")
            self.reach_posture = np.array([0.0, 2.10, -0.40, -0.15, 0.0],
                                         dtype=np.float64)
        self.reach_posture = clamp_array(
            self.reach_posture, JOINT_LIMITS[:, 0], JOINT_LIMITS[:, 1])

        self.current_joints = DEFAULT_JOINTS.copy()
        self.last_command_joints = DEFAULT_JOINTS.copy()
        self.palm_xyz = None
        self.last_palm_time = None
        self.palm_anchor_xyz = None
        self.ee_anchor_xyz = None
        self.has_control_center = False
        self.has_anchor = False
        self.control_center_mr = np.zeros(3, dtype=np.float64)
        self.ee_center_robot = np.zeros(3, dtype=np.float64)
        self.filtered_target_xyz = None
        self.r_robot_from_mr = self.make_axis_transform_matrix()
        self.last_timeout_flag = 0.0
        self.last_target_xyz = None
        self.last_error_norm = float("nan")
        self.last_clamped_flag = 0.0
        self.last_palm_delta_raw = np.zeros(3, dtype=np.float64)
        self.last_palm_delta_robot = np.zeros(3, dtype=np.float64)
        self.last_palm_now_mr = np.zeros(3, dtype=np.float64)
        self.last_scaled_delta = np.zeros(3, dtype=np.float64)
        self.last_target_raw_xyz = np.zeros(3, dtype=np.float64)
        self.last_target_before_safety_xyz = np.zeros(3, dtype=np.float64)
        self.last_target_control_xyz = np.zeros(3, dtype=np.float64)
        self.last_target_kinematics_xyz = np.zeros(3, dtype=np.float64)
        self.last_target_delta = np.zeros(3, dtype=np.float64)
        self.last_reach_radius = 0.0
        self.last_effective_reach_radius = self.effective_reach_radius
        self.last_reach_radius_clamped_flag = 0.0
        self.last_range_scale = np.ones(3, dtype=np.float64)
        self.last_palm_abs_robot = np.zeros(3, dtype=np.float64)
        self.last_r_palm = 0.0
        self.last_y_palm = 0.0
        self.last_z_palm = 0.0
        self.last_r_target = 0.0
        self.last_y_target = 0.0
        self.last_z_target = 0.0
        self.range_scaled_ignored_warned = False
        self.last_debug_log_time = 0.0
        self.last_q_current = DEFAULT_JOINTS.copy()
        self.last_q_solution = DEFAULT_JOINTS.copy()
        self.last_dq = np.zeros(5, dtype=np.float64)
        self.last_fk_before = self.fk_xyz(DEFAULT_JOINTS)
        self.last_fk_after = self.fk_xyz(DEFAULT_JOINTS)
        self.last_error_before = float("nan")
        self.last_extension_weight = 0.0
        self.last_motion_norm = 0.0
        self.last_limit_hit = 0.0
        self.last_reach_vec = np.zeros(3, dtype=np.float64)
        self.last_reach_error = 0.0
        self.last_reach_weight = 0.0
        self.last_q_reach = self.reach_posture.copy()
        self.last_desired_reach_yaw = 0.0
        self.joint_state_received = False

        self.create_subscription(PoseStamped, self.palm_pose_topic, self.palm_cb, 50)
        self.create_subscription(
            PoseStamped, self.control_center_topic, self.control_center_cb, 10)
        self.create_subscription(Bool, "/amir/reset_palm_anchor", self.reset_anchor_cb, 10)
        self.create_subscription(JointState, "/joint_states", self.joint_state_cb, 10)

        self.target_pub = self.create_publisher(PointStamped, "/amir/target_xyz", 10)
        self.joint_cmd_pub = self.create_publisher(Float32MultiArray, "/amir/joint_cmd", 10)
        self.joint_cmd_deg_pub = self.create_publisher(Float32MultiArray, "/amir/joint_cmd_deg", 10)
        self.palm_delta_raw_pub = self.create_publisher(
            Float32MultiArray, "/amir/palm_delta_raw", 10)
        self.palm_delta_robot_pub = self.create_publisher(
            Vector3Stamped, "/amir/palm_delta_robot", 10)
        self.palm_delta_mr_pub = self.create_publisher(
            Vector3Stamped, "/amir/palm_delta_mr", 10)
        self.control_center_pub = self.create_publisher(
            PointStamped, "/amir/control_center_mr", 10)
        self.ee_center_pub = self.create_publisher(
            PointStamped, "/amir/ee_center_robot", 10)
        self.target_delta_pub = self.create_publisher(
            Float32MultiArray, "/amir/target_delta", 10)
        self.ik_debug_pub = self.create_publisher(Float32MultiArray, "/amir/ik_debug", 10)
        self.metrics_pub = self.create_publisher(Float32MultiArray, "/amir_metrics", 10)
        self.trajectory_pub = None
        if self.mode in ("gazebo", "real"):
            self.trajectory_pub = self.create_publisher(
                JointTrajectory, "/arm_controller/joint_trajectory", 10)

        self.timer = self.create_timer(1.0 / self.control_rate_hz, self.control_loop)
        self.get_logger().info(
            "AmirPalmIkNode started "
            f"mode={self.mode} command_unit={self.command_unit} "
            f"rate={self.control_rate_hz:.1f}Hz "
            f"kinematics_model={self.kinematics_model} "
            f"enable_kinematics_target_adapter={self.enable_kinematics_target_adapter} "
            f"kinematics_target_adapter_mode={self.kinematics_target_adapter_mode} "
            f"urdf_source_file={URDF_SOURCE_FILE} "
            f"fk_base_link={URDF_BASE_LINK} "
            f"fk_tip_link={URDF_TIP_LINK} "
            f"joint_axes={URDF_JOINT_AXES.tolist()} "
            f"joint_origins_xyz={URDF_JOINT_ORIGINS_XYZ.tolist()} "
            f"joint_origins_rpy={URDF_JOINT_ORIGINS_RPY.tolist()} "
            f"joint_limits_source=urdf "
            f"palm_pose_topic={self.palm_pose_topic} "
            f"control_center_topic={self.control_center_topic} "
            f"target_mapping_mode={self.target_mapping_mode} "
            f"absolute_axis_mapping={self.absolute_axis_mapping} "
            f"use_control_center={self.use_control_center} "
            f"require_control_center={self.require_control_center} "
            f"scale_x={self.scale_xyz[0]:.4f} "
            f"scale_y={self.scale_xyz[1]:.4f} "
            f"scale_z={self.scale_xyz[2]:.4f} "
            f"deadzone_m={self.deadzone_m:.4f} "
            f"max_delta_norm_m={self.max_delta_norm_m:.4f} "
            f"enable_reach_radius_limit={self.enable_reach_radius_limit} "
            f"reach_radius_max_m={self.reach_radius_max_m:.4f} "
            f"reach_radius_margin_m={self.reach_radius_margin_m:.4f} "
            f"effective_reach_radius={self.effective_reach_radius:.4f} "
            f"reach_radius_origin={np.round(self.reach_radius_origin, 4).tolist()} "
            f"lowpass_alpha={self.lowpass_alpha:.4f} "
            f"input_timeout_sec={self.input_timeout_sec:.4f} "
            f"axis_map={self.palm_to_robot_axis_map.tolist()} "
            f"axis_sign={self.palm_to_robot_axis_sign.tolist()} "
            f"axis_gain={self.palm_axis_gain.tolist()} "
            f"mr_range_x=[{self.mr_min[0]:.3f}, {self.mr_max[0]:.3f}] "
            f"mr_range_y=[{self.mr_min[1]:.3f}, {self.mr_max[1]:.3f}] "
            f"mr_range_z=[{self.mr_min[2]:.3f}, {self.mr_max[2]:.3f}] "
            f"abs_robot_range_x=[{self.abs_robot_min[0]:.3f}, "
            f"{self.abs_robot_max[0]:.3f}] "
            f"abs_robot_range_y=[{self.abs_robot_min[1]:.3f}, "
            f"{self.abs_robot_max[1]:.3f}] "
            f"abs_robot_range_z=[{self.abs_robot_min[2]:.3f}, "
            f"{self.abs_robot_max[2]:.3f}] "
            f"debug_target_step_x={self.debug_target_step_x:.4f} "
            f"palm_target_mode={self.palm_target_mode} "
            f"cylindrical_r_gain={self.cylindrical_r_gain:.4f} "
            f"cylindrical_z_gain={self.cylindrical_z_gain:.4f} "
            f"cylindrical_y_gain={self.cylindrical_y_gain:.4f} "
            f"cylindrical_r_offset={self.cylindrical_r_offset:.4f} "
            f"cylindrical_z_offset={self.cylindrical_z_offset:.4f} "
            f"cylindrical_y_offset={self.cylindrical_y_offset:.4f} "
            f"cylindrical_r_range=[{self.cylindrical_min_r:.4f}, "
            f"{self.cylindrical_max_r:.4f}] "
            f"cylindrical_z_range=[{self.cylindrical_min_z:.4f}, "
            f"{self.cylindrical_max_z:.4f}] "
            f"enable_range_scaled_palm={self.enable_range_scaled_palm} "
            f"human_motion_range={self.human_motion_range.tolist()} "
            f"robot_motion_range={self.robot_motion_range.tolist()} "
            f"range_scale_gain={self.range_scale_gain:.4f} "
            f"range_scale_max={self.range_scale_max.tolist()} "
            f"enable_extension_bias={self.enable_extension_bias} "
            f"extension_bias_gain={self.extension_bias_gain:.4f} "
            f"extension_trigger_x={self.extension_trigger_x:.4f} "
            f"extension_trigger_norm={self.extension_trigger_norm:.4f} "
            f"extension_use_motion_norm={self.extension_use_motion_norm} "
            f"extension_posture={np.round(self.extension_posture, 4).tolist()} "
            f"joint_limit_avoidance_gain={self.joint_limit_avoidance_gain:.4f} "
            f"enable_reach_approach={self.enable_reach_approach} "
            f"reach_trigger_error={self.reach_trigger_error:.4f} "
            f"reach_full_error={self.reach_full_error:.4f} "
            f"reach_bias_gain={self.reach_bias_gain:.4f} "
            f"reach_posture={np.round(self.reach_posture, 4).tolist()} "
            f"reach_directional_yaw={self.reach_directional_yaw} "
            f"reach_yaw_trigger_norm={self.reach_yaw_trigger_norm:.4f}")

    def palm_cb(self, msg):
        self.palm_xyz = np.array([
            float(msg.pose.position.x),
            float(msg.pose.position.y),
            float(msg.pose.position.z),
        ], dtype=np.float64)
        self.last_palm_time = seconds_now(self)

        if self.target_mapping_mode == "absolute":
            self.last_palm_now_mr = self.palm_xyz.copy()
        elif self.use_control_center:
            if not self.has_anchor and (self.has_control_center or not self.require_control_center):
                self.set_ee_anchor_from_current_pose()
        elif self.palm_anchor_xyz is None:
            self.set_legacy_palm_anchor()

    def fk_xyz(self, joints):
        if self.kinematics_model == "urdf_like":
            return fk_xyz_urdf_like(joints)
        return fk_xyz(joints)

    def adapt_target_for_kinematics(self, target_control):
        if (self.kinematics_model != "urdf_like" or
                not self.enable_kinematics_target_adapter or
                self.kinematics_target_adapter_mode == "none"):
            return target_control.copy()
        if self.kinematics_target_adapter_mode == "control_to_urdf_y_forward":
            return np.array([
                -target_control[1],
                target_control[0],
                target_control[2],
            ], dtype=np.float64)
        return target_control.copy()

    def control_center_cb(self, msg):
        self.control_center_mr = np.array([
            float(msg.pose.position.x),
            float(msg.pose.position.y),
            float(msg.pose.position.z),
        ], dtype=np.float64)
        first_center = not self.has_control_center
        self.has_control_center = True
        if first_center:
            self.get_logger().info(
                "control center received "
                f"control_center_mr={np.round(self.control_center_mr, 4).tolist()}")

    def reset_anchor_cb(self, msg):
        if not bool(msg.data):
            return
        current_ee = self.fk_xyz(self.current_joints)
        self.has_anchor = False
        self.palm_anchor_xyz = None
        self.ee_anchor_xyz = None
        self.ee_center_robot = current_ee.copy()
        self.last_target_xyz = current_ee.copy()
        self.filtered_target_xyz = current_ee.copy()
        self.get_logger().warn(
            "palm anchor reset "
            f"current_ee_xyz={np.round(current_ee, 4).tolist()}")

    def set_ee_anchor_from_current_pose(self):
        self.ee_center_robot = self.fk_xyz(self.current_joints)
        self.ee_anchor_xyz = self.ee_center_robot.copy()
        self.last_target_xyz = self.ee_center_robot.copy()
        self.filtered_target_xyz = self.ee_center_robot.copy()
        self.has_anchor = True
        palm_delta_mr = (
            self.palm_xyz - self.control_center_mr
            if self.has_control_center else
            np.zeros(3, dtype=np.float64)
        )
        self.get_logger().info(
            "EE anchor set from first valid palm_pose "
            f"ee_center_robot={np.round(self.ee_center_robot, 4).tolist()} "
            f"palm_now_mr={np.round(self.palm_xyz, 4).tolist()} "
            f"control_center_mr={np.round(self.control_center_mr, 4).tolist()} "
            f"palm_delta_mr={np.round(palm_delta_mr, 4).tolist()}")

    def set_legacy_palm_anchor(self):
        self.palm_anchor_xyz = self.palm_xyz.copy()
        self.ee_anchor_xyz = self.fk_xyz(self.current_joints)
        self.ee_center_robot = self.ee_anchor_xyz.copy()
        self.has_anchor = True
        if self.palm_target_mode == "relative":
            self.last_target_xyz = clamp_array(
                self.ee_anchor_xyz.copy(), self.workspace_min, self.workspace_max)
            self.filtered_target_xyz = self.last_target_xyz.copy()
        else:
            self.last_target_xyz = None
            self.filtered_target_xyz = None
        self.get_logger().info(
            "Legacy palm anchor set "
            f"palm={self.palm_anchor_xyz.tolist()} ee={self.ee_anchor_xyz.tolist()} "
            f"palm_target_mode={self.palm_target_mode} "
            f"axis_map={self.palm_to_robot_axis_map.tolist()} "
            f"axis_sign={self.palm_to_robot_axis_sign.tolist()} "
            f"palm_motion_gain={self.palm_motion_gain:.4f} "
            f"max_target_step_m={self.max_target_step_m:.4f} "
            f"trajectory_time_from_start_sec={self.trajectory_time_from_start_sec:.4f}")

    def joint_state_cb(self, msg):
        name_to_idx = {name: idx for idx, name in enumerate(msg.name)}
        updated = self.current_joints.copy()
        for joint_idx, joint_name in enumerate(JOINT_NAMES):
            if joint_name in name_to_idx:
                updated[joint_idx] = float(msg.position[name_to_idx[joint_name]])
        self.current_joints = clamp_array(updated, JOINT_LIMITS[:, 0], JOINT_LIMITS[:, 1])
        self.joint_state_received = True
        if not self.has_anchor:
            self.last_command_joints = self.current_joints.copy()

    def make_axis_transform_matrix(self):
        matrix = np.zeros((3, 3), dtype=np.float64)
        for robot_axis, mr_axis in enumerate(self.palm_to_robot_axis_map):
            matrix[robot_axis, int(mr_axis)] = self.palm_to_robot_axis_sign[robot_axis]
        return matrix

    def convert_palm_delta_to_robot_delta(self, palm_delta_raw):
        mapped = palm_delta_raw[self.palm_to_robot_axis_map]
        return self.palm_to_robot_axis_sign * self.palm_axis_gain * mapped

    def convert_palm_xyz_to_robot_abs(self, palm_xyz):
        mapped = palm_xyz[self.palm_to_robot_axis_map]
        return self.palm_to_robot_axis_sign * mapped

    def palm_delta_to_target_delta(self, palm_delta_robot):
        if not self.enable_range_scaled_palm:
            self.last_range_scale = np.ones(3, dtype=np.float64)
            return self.palm_motion_gain * palm_delta_robot

        scale = self.robot_motion_range / self.human_motion_range
        scale = np.clip(scale, 0.0, self.range_scale_max)
        self.last_range_scale = scale
        return self.range_scale_gain * scale * palm_delta_robot

    def limit_target_step(self, clamped_target, clamped_flag):
        if self.last_target_xyz is None:
            limited_target = clamped_target
        else:
            delta = clamped_target - self.last_target_xyz
            delta_norm = float(np.linalg.norm(delta))
            if self.max_target_step_m > 0.0 and delta_norm > self.max_target_step_m:
                delta = delta * (self.max_target_step_m / delta_norm)
                clamped_flag = 1.0
            limited_target = self.last_target_xyz + delta

        limited_target = clamp_array(limited_target, self.workspace_min, self.workspace_max)
        self.last_target_xyz = limited_target
        self.last_clamped_flag = clamped_flag
        return limited_target

    def make_target(self):
        if self.target_mapping_mode == "absolute":
            return self.make_absolute_target()
        if self.use_control_center:
            return self.make_control_center_target()
        if self.palm_target_mode == "cylindrical_absolute":
            return self.make_cylindrical_absolute_target()
        return self.make_relative_target()

    def apply_target_safety(self, target_raw, clamped_flag=0.0):
        target_raw_before_safety = target_raw.copy()
        target_safety, reach_radius_clamped = self.apply_reach_radius_limit(target_raw)
        clamped_flag = max(clamped_flag, reach_radius_clamped)

        workspace_limited = clamp_array(target_safety, self.workspace_min, self.workspace_max)
        if not np.allclose(target_safety, workspace_limited):
            clamped_flag = 1.0
            self.get_logger().warn(
                "target clamp applied "
                f"target_raw={np.round(target_safety, 4).tolist()} "
                f"workspace_limited={np.round(workspace_limited, 4).tolist()}",
                throttle_duration_sec=1.0)

        target_limited = self.limit_target_step(workspace_limited, clamped_flag)
        if self.filtered_target_xyz is None:
            self.filtered_target_xyz = target_limited.copy()
        else:
            self.filtered_target_xyz = (
                self.lowpass_alpha * target_limited +
                (1.0 - self.lowpass_alpha) * self.filtered_target_xyz
            )

        self.last_target_raw_xyz = target_raw_before_safety.copy()
        self.last_target_before_safety_xyz = target_raw_before_safety.copy()
        self.last_clamped_flag = max(self.last_clamped_flag, clamped_flag)
        return self.filtered_target_xyz.copy()

    def make_absolute_target(self):
        palm_now_mr = self.palm_xyz.copy()
        target_raw = np.array([
            map_range_clamped(
                palm_now_mr[2],
                self.mr_min[2],
                self.mr_max[2],
                self.abs_robot_min[0],
                self.abs_robot_max[0]),
            map_range_clamped(
                palm_now_mr[0],
                self.mr_min[0],
                self.mr_max[0],
                self.abs_robot_max[1],
                self.abs_robot_min[1]),
            map_range_clamped(
                palm_now_mr[1],
                self.mr_min[1],
                self.mr_max[1],
                self.abs_robot_min[2],
                self.abs_robot_max[2]),
        ], dtype=np.float64)

        filtered_target = self.apply_target_safety(target_raw)
        self.last_palm_now_mr = palm_now_mr.copy()
        self.last_palm_delta_raw = np.zeros(3, dtype=np.float64)
        self.last_palm_delta_robot = np.zeros(3, dtype=np.float64)
        self.last_scaled_delta = np.zeros(3, dtype=np.float64)
        self.last_target_delta = filtered_target - self.fk_xyz(self.current_joints)
        self.last_motion_norm = 0.0
        return filtered_target

    def apply_reach_radius_limit(self, target_xyz):
        self.last_reach_radius_clamped_flag = 0.0
        self.last_effective_reach_radius = self.effective_reach_radius
        p = target_xyz - self.reach_radius_origin
        radius = float(np.linalg.norm(p))
        self.last_reach_radius = radius

        if (not self.enable_reach_radius_limit or
                self.effective_reach_radius <= 0.0 or
                radius <= self.effective_reach_radius):
            return target_xyz.copy(), 0.0

        limited = self.reach_radius_origin + p * (self.effective_reach_radius / radius)
        self.last_reach_radius_clamped_flag = 1.0
        self.get_logger().warn(
            "reach radius clamp applied "
            f"r={radius:.4f} effective_radius={self.effective_reach_radius:.4f}",
            throttle_duration_sec=1.0)
        return limited, 1.0

    def make_control_center_target(self):
        palm_delta_mr = self.palm_xyz - self.control_center_mr
        palm_delta_norm = float(np.linalg.norm(palm_delta_mr))
        if palm_delta_norm < self.deadzone_m:
            palm_delta_mr = np.zeros(3, dtype=np.float64)
            palm_delta_norm = 0.0

        palm_delta_robot = self.r_robot_from_mr @ palm_delta_mr
        robot_delta_norm = float(np.linalg.norm(palm_delta_robot))
        if abs(robot_delta_norm - palm_delta_norm) > 1e-6:
            self.get_logger().warn(
                "palm_delta norm mismatch after R_robot_from_mr "
                f"palm_delta_norm={palm_delta_norm:.6f} "
                f"robot_delta_norm={robot_delta_norm:.6f}",
                throttle_duration_sec=1.0)
        scaled_delta = self.scale_xyz * palm_delta_robot
        target_raw = self.ee_center_robot + self.palm_motion_gain * scaled_delta
        target_raw_unclamped = target_raw.copy()

        clamped_flag = 0.0
        center_delta = target_raw - self.ee_center_robot
        center_delta_norm = float(np.linalg.norm(center_delta))
        if self.max_delta_norm_m > 0.0 and center_delta_norm > self.max_delta_norm_m:
            target_raw = self.ee_center_robot + center_delta * (
                self.max_delta_norm_m / center_delta_norm)
            clamped_flag = 1.0

        filtered_target = self.apply_target_safety(target_raw, clamped_flag)

        self.last_palm_delta_raw = palm_delta_mr.copy()
        self.last_palm_delta_robot = palm_delta_robot.copy()
        self.last_palm_now_mr = self.palm_xyz.copy()
        self.last_scaled_delta = scaled_delta.copy()
        self.last_target_raw_xyz = target_raw_unclamped.copy()
        self.last_target_before_safety_xyz = target_raw_unclamped.copy()
        self.last_target_delta = filtered_target - self.ee_center_robot
        self.last_motion_norm = palm_delta_norm
        return filtered_target

    def make_relative_target(self):
        palm_delta_raw = self.palm_xyz - self.palm_anchor_xyz
        palm_delta_robot = self.convert_palm_delta_to_robot_delta(palm_delta_raw)
        target_delta = self.palm_delta_to_target_delta(palm_delta_robot)
        raw_target = self.ee_anchor_xyz + target_delta
        raw_target[0] += self.debug_target_step_x

        self.last_palm_delta_raw = palm_delta_raw
        self.last_palm_delta_robot = palm_delta_robot

        clamped_target = clamp_array(raw_target, self.workspace_min, self.workspace_max)
        clamped_flag = 1.0 if not np.allclose(raw_target, clamped_target) else 0.0

        limited_target = self.limit_target_step(clamped_target, clamped_flag)
        self.last_target_delta = limited_target - self.ee_anchor_xyz
        return limited_target

    def make_cylindrical_absolute_target(self):
        if self.enable_range_scaled_palm and not self.range_scaled_ignored_warned:
            self.get_logger().warn(
                "enable_range_scaled_palm is ignored when "
                "palm_target_mode=cylindrical_absolute.")
            self.range_scaled_ignored_warned = True
        self.last_range_scale = np.ones(3, dtype=np.float64)

        if self.palm_anchor_xyz is not None:
            palm_delta_raw = self.palm_xyz - self.palm_anchor_xyz
            palm_delta_robot = self.convert_palm_delta_to_robot_delta(palm_delta_raw)
        else:
            palm_delta_raw = np.zeros(3, dtype=np.float64)
            palm_delta_robot = np.zeros(3, dtype=np.float64)
        self.last_palm_delta_raw = palm_delta_raw
        self.last_palm_delta_robot = palm_delta_robot

        palm_abs_robot = self.convert_palm_xyz_to_robot_abs(self.palm_xyz)
        r_palm = float(math.hypot(float(palm_abs_robot[0]), float(palm_abs_robot[1])))
        y_palm = float(palm_abs_robot[1])
        z_palm = float(palm_abs_robot[2])

        r_target = float(np.clip(
            self.cylindrical_r_gain * r_palm + self.cylindrical_r_offset,
            self.cylindrical_min_r,
            self.cylindrical_max_r))
        z_target = float(np.clip(
            self.cylindrical_z_gain * z_palm + self.cylindrical_z_offset,
            self.cylindrical_min_z,
            self.cylindrical_max_z))
        y_target = self.cylindrical_y_gain * y_palm + self.cylindrical_y_offset

        raw_target = np.array([r_target, y_target, z_target], dtype=np.float64)
        raw_target[0] += self.debug_target_step_x

        self.last_palm_abs_robot = palm_abs_robot.copy()
        self.last_r_palm = r_palm
        self.last_y_palm = y_palm
        self.last_z_palm = z_palm
        self.last_r_target = r_target
        self.last_y_target = float(y_target)
        self.last_z_target = z_target

        clamped_target = clamp_array(raw_target, self.workspace_min, self.workspace_max)
        clamped_flag = 1.0 if not np.allclose(raw_target, clamped_target) else 0.0
        limited_target = self.limit_target_step(clamped_target, clamped_flag)
        if self.ee_anchor_xyz is not None:
            self.last_target_delta = limited_target - self.ee_anchor_xyz
        else:
            self.last_target_delta = np.zeros(3, dtype=np.float64)
        return limited_target

    def make_debug_target_from_joint_state(self):
        self.ee_anchor_xyz = self.fk_xyz(self.current_joints)
        self.last_palm_delta_raw = np.zeros(3, dtype=np.float64)
        self.last_palm_delta_robot = np.zeros(3, dtype=np.float64)
        self.last_palm_abs_robot = np.zeros(3, dtype=np.float64)
        self.last_r_palm = 0.0
        self.last_y_palm = 0.0
        self.last_z_palm = 0.0
        self.last_r_target = 0.0
        self.last_y_target = 0.0
        self.last_z_target = 0.0
        self.last_range_scale = (
            np.clip(
                self.robot_motion_range / self.human_motion_range,
                0.0,
                self.range_scale_max)
            if self.enable_range_scaled_palm else
            np.ones(3, dtype=np.float64)
        )

        raw_target = self.ee_anchor_xyz + np.array(
            [self.debug_target_step_x, 0.0, 0.0], dtype=np.float64)
        target_xyz = clamp_array(raw_target, self.workspace_min, self.workspace_max)

        self.last_target_xyz = target_xyz.copy()
        self.last_target_delta = target_xyz - self.ee_anchor_xyz
        self.last_clamped_flag = 1.0 if not np.allclose(raw_target, target_xyz) else 0.0
        return target_xyz

    def publish_float_array(self, publisher, values):
        msg = Float32MultiArray()
        msg.data = [float(v) for v in values]
        publisher.publish(msg)

    def make_point_stamped(self, xyz, frame_id="amir_base"):
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.point.x = float(xyz[0])
        msg.point.y = float(xyz[1])
        msg.point.z = float(xyz[2])
        return msg

    def make_vector3_stamped(self, xyz, frame_id="amir_base"):
        msg = Vector3Stamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.vector.x = float(xyz[0])
        msg.vector.y = float(xyz[1])
        msg.vector.z = float(xyz[2])
        return msg

    def publish_joint_outputs(self, joints_rad):
        if self.command_unit == "mrad":
            joint_cmd = joints_rad * 1000.0
        else:
            joint_cmd = joints_rad
        self.publish_float_array(self.joint_cmd_pub, joint_cmd)
        self.publish_float_array(self.joint_cmd_deg_pub, np.degrees(joints_rad))

    def publish_debug_deltas(self):
        if self.use_control_center:
            self.last_palm_delta_robot = self.r_robot_from_mr @ self.last_palm_delta_raw
        self.publish_float_array(self.palm_delta_raw_pub, self.last_palm_delta_raw)
        self.palm_delta_mr_pub.publish(self.make_vector3_stamped(self.last_palm_delta_raw))
        self.palm_delta_robot_pub.publish(
            self.make_vector3_stamped(self.last_palm_delta_robot))
        self.control_center_pub.publish(self.make_point_stamped(self.control_center_mr))
        self.ee_center_pub.publish(self.make_point_stamped(self.ee_center_robot))
        self.publish_float_array(self.target_delta_pub, self.last_target_delta)

    def extension_weight(self):
        if not self.enable_extension_bias:
            self.last_motion_norm = float(np.linalg.norm(self.last_target_delta))
            return 0.0

        if self.extension_use_motion_norm:
            motion = float(np.linalg.norm(self.last_target_delta))
            trigger = self.extension_trigger_norm
        else:
            motion = float(self.last_target_delta[0])
            trigger = self.extension_trigger_x

        self.last_motion_norm = float(np.linalg.norm(self.last_target_delta))
        if motion <= trigger:
            return 0.0
        if trigger <= 1e-6:
            return 1.0
        return float(np.clip((motion - trigger) / trigger, 0.0, 1.0))

    def extension_bias_delta(self, joints_rad):
        weight = self.extension_weight()
        self.last_extension_weight = weight
        if weight <= 0.0 or self.extension_bias_gain <= 0.0:
            return np.zeros(5, dtype=np.float64)

        bias_mask = np.array([0.0, 1.0, 1.0, 1.0, 0.0], dtype=np.float64)
        return (
            self.extension_bias_gain * weight * bias_mask *
            (self.extension_posture - joints_rad)
        )

    def reach_weight(self, reach_error):
        if not self.enable_reach_approach:
            return 0.0
        if reach_error <= self.reach_trigger_error:
            return 0.0
        denom = max(self.reach_full_error - self.reach_trigger_error, 1e-6)
        return float(np.clip((reach_error - self.reach_trigger_error) / denom, 0.0, 1.0))

    def reach_approach_delta(self, joints_rad, q_current, reach_vec, reach_error):
        weight = self.reach_weight(reach_error)
        self.last_reach_weight = weight
        self.last_reach_error = float(reach_error)
        self.last_reach_vec = reach_vec.copy()
        self.last_q_reach = self.reach_posture.copy()
        self.last_desired_reach_yaw = float(q_current[0])

        if weight <= 0.0 or self.reach_bias_gain <= 0.0:
            return np.zeros(5, dtype=np.float64)

        q_reach = self.reach_posture.copy()
        bias_mask = np.array([0.0, 1.0, 1.0, 1.0, 0.0], dtype=np.float64)
        reach_xy_norm = float(np.linalg.norm(reach_vec[:2]))
        if self.reach_directional_yaw and reach_xy_norm > self.reach_yaw_trigger_norm:
            desired_yaw = math.atan2(float(reach_vec[1]), float(reach_vec[0]))
            q_reach[0] = float(np.clip(
                desired_yaw, JOINT_LIMITS[0, 0], JOINT_LIMITS[0, 1]))
            bias_mask[0] = 1.0
            self.last_desired_reach_yaw = q_reach[0]
        else:
            q_reach[0] = float(q_current[0])
            self.last_desired_reach_yaw = float(q_current[0])

        self.last_q_reach = q_reach.copy()
        return self.reach_bias_gain * weight * bias_mask * (q_reach - joints_rad)

    def joint_limit_avoidance_delta(self, joints_rad):
        if self.joint_limit_avoidance_gain <= 0.0:
            self.last_limit_hit = 0.0
            return np.zeros(5, dtype=np.float64)

        lower = JOINT_LIMITS[:, 0]
        upper = JOINT_LIMITS[:, 1]
        center = 0.5 * (lower + upper)
        span = np.maximum(upper - lower, 1e-6)
        margin_ratio = np.minimum(joints_rad - lower, upper - joints_rad) / span
        threshold = 0.20
        proximity = np.clip((threshold - margin_ratio) / threshold, 0.0, 1.0)
        self.last_limit_hit = 1.0 if np.any(proximity > 0.0) else 0.0
        return self.joint_limit_avoidance_gain * proximity * (center - joints_rad)

    def apply_posture_biases(self, joints_rad, target_xyz, base_error, q_current, fk_before):
        reach_vec = target_xyz - fk_before
        reach_error = float(np.linalg.norm(reach_vec))
        reach_delta = self.reach_approach_delta(
            joints_rad, q_current, reach_vec, reach_error)
        if float(np.linalg.norm(reach_delta)) > 1e-12:
            # Reach approach owns the posture preference for this cycle.
            self.last_extension_weight = 0.0
            posture_delta = reach_delta
        else:
            posture_delta = self.extension_bias_delta(joints_rad)
            self.last_reach_weight = 0.0
            self.last_reach_error = reach_error
            self.last_reach_vec = reach_vec.copy()
            self.last_q_reach = self.reach_posture.copy()
            self.last_desired_reach_yaw = float(q_current[0])

        bias_delta = posture_delta + self.joint_limit_avoidance_delta(joints_rad)
        if float(np.linalg.norm(bias_delta)) <= 1e-12:
            return joints_rad

        # Keep the posture preference subordinate to the hand-position task.
        max_allowed_error = base_error + max(0.005, 0.5 * base_error)
        for scale in (1.0, 0.5, 0.25, 0.1):
            candidate = clamp_array(
                joints_rad + scale * bias_delta,
                JOINT_LIMITS[:, 0],
                JOINT_LIMITS[:, 1])
            candidate_error = float(np.linalg.norm(target_xyz - self.fk_xyz(candidate)))
            if candidate_error <= max_allowed_error:
                self.last_extension_weight *= scale
                self.last_reach_weight *= scale
                return candidate

        self.last_extension_weight = 0.0
        self.last_reach_weight = 0.0
        return joints_rad

    def publish_ik_debug(self):
        self.publish_float_array(self.ik_debug_pub, [
            self.last_error_before,
            self.last_error_norm,
            self.last_dq[0],
            self.last_dq[1],
            self.last_dq[2],
            self.last_dq[3],
            self.last_dq[4],
            self.last_target_delta[0],
            self.last_target_delta[1],
            self.last_target_delta[2],
            self.last_fk_before[0],
            self.last_fk_before[1],
            self.last_fk_before[2],
            self.last_fk_after[0],
            self.last_fk_after[1],
            self.last_fk_after[2],
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
            self.last_dq[0],
            self.last_dq[1],
            self.last_dq[2],
            self.last_dq[3],
            self.last_dq[4],
            self.last_limit_hit,
            self.last_extension_weight,
            self.last_error_before,
            self.last_error_norm,
            1.0 if self.extension_use_motion_norm else 0.0,
            self.extension_trigger_norm,
            self.last_motion_norm,
            self.last_extension_weight,
            1.0 if self.enable_reach_approach else 0.0,
            self.last_reach_error,
            self.last_reach_weight,
            self.last_reach_vec[0],
            self.last_reach_vec[1],
            self.last_reach_vec[2],
            self.last_q_reach[0],
            self.last_q_reach[1],
            self.last_q_reach[2],
            self.last_q_reach[3],
            self.last_q_reach[4],
            self.reach_bias_gain,
            1.0 if self.reach_directional_yaw else 0.0,
            self.last_desired_reach_yaw,
            1.0 if self.enable_range_scaled_palm else 0.0,
            self.last_range_scale[0],
            self.last_range_scale[1],
            self.last_range_scale[2],
            1.0 if self.palm_target_mode == "cylindrical_absolute" else 0.0,
            self.last_palm_abs_robot[0],
            self.last_palm_abs_robot[1],
            self.last_palm_abs_robot[2],
            self.last_r_palm,
            self.last_y_palm,
            self.last_z_palm,
            self.last_r_target,
            self.last_y_target,
            self.last_z_target,
            self.last_target_xyz[0] if self.last_target_xyz is not None else 0.0,
            self.last_target_xyz[1] if self.last_target_xyz is not None else 0.0,
            self.last_target_xyz[2] if self.last_target_xyz is not None else 0.0,
            self.last_palm_now_mr[0],
            self.last_palm_now_mr[1],
            self.last_palm_now_mr[2],
            self.control_center_mr[0],
            self.control_center_mr[1],
            self.control_center_mr[2],
            self.last_palm_delta_raw[0],
            self.last_palm_delta_raw[1],
            self.last_palm_delta_raw[2],
            self.last_palm_delta_robot[0],
            self.last_palm_delta_robot[1],
            self.last_palm_delta_robot[2],
            self.last_scaled_delta[0],
            self.last_scaled_delta[1],
            self.last_scaled_delta[2],
            self.last_target_raw_xyz[0],
            self.last_target_raw_xyz[1],
            self.last_target_raw_xyz[2],
            self.palm_to_robot_axis_map[0],
            self.palm_to_robot_axis_map[1],
            self.palm_to_robot_axis_map[2],
            self.palm_to_robot_axis_sign[0],
            self.palm_to_robot_axis_sign[1],
            self.palm_to_robot_axis_sign[2],
            self.last_reach_radius,
            self.last_effective_reach_radius,
            self.last_reach_radius_clamped_flag,
            1.0 if self.target_mapping_mode == "absolute" else 0.0,
            1.0 if self.target_mapping_mode == "absolute" else 0.0,
            self.mr_min[0],
            self.mr_max[0],
            self.mr_min[1],
            self.mr_max[1],
            self.mr_min[2],
            self.mr_max[2],
            self.abs_robot_min[0],
            self.abs_robot_max[0],
            self.abs_robot_min[1],
            self.abs_robot_max[1],
            self.abs_robot_min[2],
            self.abs_robot_max[2],
            self.last_target_before_safety_xyz[0],
            self.last_target_before_safety_xyz[1],
            self.last_target_before_safety_xyz[2],
            self.last_target_control_xyz[0],
            self.last_target_control_xyz[1],
            self.last_target_control_xyz[2],
            self.last_target_kinematics_xyz[0],
            self.last_target_kinematics_xyz[1],
            self.last_target_kinematics_xyz[2],
            1.0 if self.enable_kinematics_target_adapter else 0.0,
            1.0 if self.kinematics_target_adapter_mode == "control_to_urdf_y_forward" else 0.0,
        ])

    def log_debug_state(self, now, target_xyz):
        if now - self.last_debug_log_time < 1.0:
            return
        self.last_debug_log_time = now
        self.get_logger().info(
            "palm_ik_debug "
            f"palm_now_mr={np.round(self.last_palm_now_mr, 4).tolist()} "
            f"control_center_mr={np.round(self.control_center_mr, 4).tolist()} "
            f"palm_delta_raw={np.round(self.last_palm_delta_raw, 4).tolist()} "
            f"palm_delta_robot={np.round(self.last_palm_delta_robot, 4).tolist()} "
            f"scaled_delta={np.round(self.last_scaled_delta, 4).tolist()} "
            f"target_raw_xyz={np.round(self.last_target_raw_xyz, 4).tolist()} "
            f"target_raw_before_safety={np.round(self.last_target_before_safety_xyz, 4).tolist()} "
            f"target_xyz_control_frame={np.round(self.last_target_control_xyz, 4).tolist()} "
            f"target_xyz_kinematics_frame={np.round(self.last_target_kinematics_xyz, 4).tolist()} "
            f"enable_kinematics_target_adapter={self.enable_kinematics_target_adapter} "
            f"kinematics_target_adapter_mode={self.kinematics_target_adapter_mode} "
            f"reach_radius={self.last_reach_radius:.4f} "
            f"effective_reach_radius={self.last_effective_reach_radius:.4f} "
            f"reach_radius_clamped={self.last_reach_radius_clamped_flag:.0f} "
            f"target_mapping_mode={self.target_mapping_mode} "
            f"absolute_mapping_enabled={int(self.target_mapping_mode == 'absolute')} "
            f"absolute_axis_mapping={self.absolute_axis_mapping} "
            f"palm_target_mode={self.palm_target_mode} "
            f"palm_abs_robot={np.round(self.last_palm_abs_robot, 4).tolist()} "
            f"r_palm={self.last_r_palm:.4f} "
            f"y_palm={self.last_y_palm:.4f} "
            f"z_palm={self.last_z_palm:.4f} "
            f"r_target={self.last_r_target:.4f} "
            f"y_target={self.last_y_target:.4f} "
            f"z_target={self.last_z_target:.4f} "
            f"target_xyz={np.round(target_xyz, 4).tolist()} "
            f"axis_map={self.palm_to_robot_axis_map.tolist()} "
            f"axis_sign={self.palm_to_robot_axis_sign.tolist()} "
            f"palm_motion_gain={self.palm_motion_gain:.4f} "
            f"max_target_step_m={self.max_target_step_m:.4f} "
            f"trajectory_time_from_start_sec={self.trajectory_time_from_start_sec:.4f} "
            f"debug_target_step_x={self.debug_target_step_x:.4f} "
            f"q_current={np.round(self.last_q_current, 4).tolist()} "
            f"q_solution={np.round(self.last_q_solution, 4).tolist()} "
            f"dq={np.round(self.last_dq, 4).tolist()} "
            f"fk_before={np.round(self.last_fk_before, 4).tolist()} "
            f"fk_after={np.round(self.last_fk_after, 4).tolist()} "
            f"target_delta={np.round(self.last_target_delta, 4).tolist()} "
            f"error_before={self.last_error_before:.5f} "
            f"error_after={self.last_error_norm:.5f} "
            f"extension_weight={self.last_extension_weight:.3f} "
            f"extension_use_motion_norm={self.extension_use_motion_norm} "
            f"extension_trigger_norm={self.extension_trigger_norm:.4f} "
            f"motion_norm={self.last_motion_norm:.4f} "
            f"enable_range_scaled_palm={self.enable_range_scaled_palm} "
            f"human_motion_range={self.human_motion_range.tolist()} "
            f"robot_motion_range={self.robot_motion_range.tolist()} "
            f"range_scale_gain={self.range_scale_gain:.4f} "
            f"range_scale={np.round(self.last_range_scale, 4).tolist()} "
            f"enable_reach_approach={self.enable_reach_approach} "
            f"reach_error={self.last_reach_error:.4f} "
            f"reach_weight={self.last_reach_weight:.3f} "
            f"reach_vec={np.round(self.last_reach_vec, 4).tolist()} "
            f"q_reach={np.round(self.last_q_reach, 4).tolist()} "
            f"reach_bias_gain={self.reach_bias_gain:.4f} "
            f"reach_directional_yaw={self.reach_directional_yaw} "
            f"desired_reach_yaw={self.last_desired_reach_yaw:.4f} "
            f"limit_hit={self.last_limit_hit:.0f}")

    def publish_trajectory(self, joints_rad):
        if self.mode not in ("gazebo", "real") or self.trajectory_pub is None:
            return

        msg = JointTrajectory()
        msg.joint_names = JOINT_NAMES
        point = JointTrajectoryPoint()
        if self.mode == "real" and self.command_unit == "mrad":
            point.positions = [float(v) for v in joints_rad * 1000.0]
        else:
            point.positions = [float(v) for v in joints_rad]
        point.time_from_start = duration_from_seconds(self.trajectory_time_from_start_sec)
        msg.points = [point]
        self.trajectory_pub.publish(msg)

    def publish_metrics(self, palm_delta_norm=0.0, timeout_flag=0.0):
        self.publish_float_array(self.metrics_pub, [
            self.last_error_norm,
            palm_delta_norm,
            self.last_clamped_flag,
            timeout_flag,
            1.0 if self.has_anchor else 0.0,
            1.0 if self.has_control_center else 0.0,
            self.last_reach_radius,
            self.last_effective_reach_radius,
            self.last_reach_radius_clamped_flag,
            1.0 if self.target_mapping_mode == "absolute" else 0.0,
        ])

    def control_loop(self):
        now = seconds_now(self)
        debug_without_palm = (
            abs(self.debug_target_step_x) > 1e-9 and
            self.joint_state_received and
            self.last_palm_time is None
        )

        if self.last_palm_time is None and not debug_without_palm:
            self.publish_metrics(0.0, 0.0)
            return

        target_age = 0.0 if debug_without_palm else now - self.last_palm_time
        input_timeout = self.input_timeout_sec if self.use_control_center else self.target_timeout_sec
        if not debug_without_palm and target_age > input_timeout:
            self.last_timeout_flag = 1.0
            self.get_logger().warn(
                "palm_pose timeout; holding command output "
                f"age={target_age:.3f}s input_timeout_sec={input_timeout:.3f}s",
                throttle_duration_sec=1.0)
            self.publish_metrics(float(np.linalg.norm(self.last_palm_delta_raw)), 1.0)
            return
        self.last_timeout_flag = 0.0

        if (self.target_mapping_mode != "absolute" and
                self.use_control_center and self.require_control_center and
                not self.has_control_center):
            self.get_logger().warn(
                "control center not received; arm command inactive",
                throttle_duration_sec=1.0)
            self.publish_metrics(float(np.linalg.norm(self.last_palm_delta_raw)), 0.0)
            return

        if debug_without_palm:
            target_xyz = self.make_debug_target_from_joint_state()
        elif (self.target_mapping_mode != "absolute" and
              self.use_control_center and not self.has_anchor):
            if self.palm_xyz is not None and (self.has_control_center or not self.require_control_center):
                self.set_ee_anchor_from_current_pose()
            else:
                self.publish_metrics(float(np.linalg.norm(self.last_palm_delta_raw)), 0.0)
                return
            target_xyz = self.make_target()
        elif (self.target_mapping_mode != "absolute" and
              not self.use_control_center and
              (self.palm_anchor_xyz is None or self.ee_anchor_xyz is None)):
            self.publish_metrics(float(np.linalg.norm(self.last_palm_delta_raw)), 0.0)
            return
        else:
            target_xyz = self.make_target()

        target_xyz_control = target_xyz.copy()
        target_xyz_kinematics = self.adapt_target_for_kinematics(target_xyz_control)
        self.last_target_control_xyz = target_xyz_control.copy()
        self.last_target_kinematics_xyz = target_xyz_kinematics.copy()

        q_current = (
            self.current_joints.copy()
            if self.joint_state_received else
            self.last_command_joints.copy()
        )
        fk_before = self.fk_xyz(q_current)
        if self.palm_target_mode == "cylindrical_absolute" and not debug_without_palm:
            self.last_target_delta = target_xyz_control - fk_before
        error_before = float(np.linalg.norm(target_xyz_kinematics - fk_before))
        seed = self.last_command_joints.copy()
        joints_rad, base_error_after = lm_ik(
            target_xyz_kinematics, seed, fk_func=self.fk_xyz)
        joints_rad = self.apply_posture_biases(
            joints_rad, target_xyz_kinematics, base_error_after, q_current, fk_before)
        fk_after = self.fk_xyz(joints_rad)
        error_norm = float(np.linalg.norm(target_xyz_kinematics - fk_after))
        if base_error_after > 0.03:
            self.get_logger().warn(
                "IK may not have converged "
                f"base_error_after={base_error_after:.5f} "
                f"target_control={np.round(target_xyz_control, 4).tolist()} "
                f"target_kinematics={np.round(target_xyz_kinematics, 4).tolist()}",
                throttle_duration_sec=1.0)
        dq = joints_rad - q_current

        self.last_command_joints = joints_rad.copy()
        if not self.joint_state_received:
            self.current_joints = joints_rad.copy()
        self.last_error_norm = error_norm
        self.last_q_current = q_current.copy()
        self.last_q_solution = joints_rad.copy()
        self.last_dq = dq.copy()
        self.last_fk_before = fk_before.copy()
        self.last_fk_after = fk_after.copy()
        self.last_error_before = error_before

        self.target_pub.publish(self.make_point_stamped(target_xyz_control))
        self.publish_debug_deltas()
        self.publish_ik_debug()
        self.publish_joint_outputs(joints_rad)
        self.publish_metrics(float(np.linalg.norm(self.last_palm_delta_raw)), 0.0)
        self.publish_trajectory(joints_rad)
        self.log_debug_state(now, target_xyz_control)


def main(args=None):
    rclpy.init(args=args)
    node = AmirPalmIkNode()
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
