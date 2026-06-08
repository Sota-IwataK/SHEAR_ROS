#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Amir用軌道制御ノード (ROS2 Humble)
# Amirの正しい運動学を使用してpalm_poseを追従
# Unit contract:
# - internal unit: rad
# - sim output: rad
# - real output: mrad if required by Amir driver

import math
import numpy as np
from enum import Enum

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, Float32MultiArray, String
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from amir_interfaces.msg import AmirCmd

# ================== Amirの実寸法 ====================
# ベース～J2の固定垂直オフセット [m]
L1_OFFSET = 0.248

# 動くリンク長 [m]（J2～J3, J3～J4, J4～先端）
DEFAULT_L = [0.310, 0.310, 0.148]

JOINT_NAMES = ['Joint_1', 'Joint_2', 'Joint_3', 'Joint_4', 'Joint_5']

# Matches amir_driver::AmirHardwareInterfaces::initial_position.
# TODO: Confirm these offsets on the real robot before enabling direct AmirCmd output.
AMIR_DRIVER_INITIAL_POSITION_MRAD = [2970.0, 2360.0, -2790.0, 1310.0, -2760.0, 260.0]

# 把持向けyawロック
YAW_LOCK_DEG = 45.0

# IK関連
SIGMA_MIN_THR = 0.03
COND_MAX_THR  = 60.0
ERR_THRESH    = 0.01

# ヌル空間関連
W_MANI  = 1.0
W_LIMIT = 0.3
K_NULL  = 0.005

# Amir関節リミット（IK内部角: th0=J2相当, th1=J3相当, th2=J4相当）
# joint_2 = 1.5708 - th0 → th0の範囲: [-0.785, 1.5708] で joint_2=[0, 2.356]に収まる
JOINT_LIMITS = [
    (-0.785,    1.5708),     # th0: J2相当 joint_2=[0, 2.356]に対応
    (-2.792527, 0.0),        # th1: J3相当 0°～-160°（実機仕様）
    (-2.094395, 1.308997),   # th2: J4相当 -120°～+75°（実機仕様）
]

# プレグラスプ関連
PREGRASP_OFFSET = 0.06
D_IN        = 0.18
D_OUT       = 0.22
SIG_K       = 40.0
BETA_SMOOTH = 0.2

# FSMパラメータ
class Phase(Enum):
    TRACK    = 0
    BACKOFF  = 1
    REALIGN  = 2
    APPROACH = 3

BACKOFF_D          = 0.05
MARGIN_MIN_THR     = 0.08
REALIGN_DT_MAX     = 0.6
IMPROVE_H_MIN      = 0.08
CD_SWITCH          = 0.5
APPROACH_LOCK_BETA = True

# ================== ユーティリティ ====================
def DegToRad(th): return (np.pi / 180.0) * th
def RadToDeg(th): return (180.0 / np.pi) * th

def angle_wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi

def as_bool(value):
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')

# ================== Amir順運動学 (FK) ====================
def fk_3d(j1, th0, th1, th2, L=DEFAULT_L, l1=L1_OFFSET):
    """
    Amirの3D順運動学
    j1  : J1関節角 [rad]（水平旋回）
    th0 : J2相当角 [rad]（肩）
    th1 : J3相当角 [rad]（肘）
    th2 : J4相当角 [rad]（手首）
    戻り値: (x, y, z) [m]  ベース座標系（z=0がベース足元）
    """
    r = (L[0]*math.sin(th0)
       + L[1]*math.sin(th0+th1)
       + L[2]*math.sin(th0+th1+th2))
    z_from_j2 = (L[0]*math.cos(th0)
               + L[1]*math.cos(th0+th1)
               + L[2]*math.cos(th0+th1+th2))
    z = l1 + z_from_j2          # ← L1_OFFSETを足してベース基準に変換
    x = r * math.cos(j1)
    y = r * math.sin(j1)
    return x, y, z

def fk_rz(th0, th1, th2, L=DEFAULT_L):
    """
    r-z平面内FK（IK計算用、L1_OFFSETを含まない）
    """
    r = (L[0]*math.sin(th0)
       + L[1]*math.sin(th0+th1)
       + L[2]*math.sin(th0+th1+th2))
    z = (L[0]*math.cos(th0)
       + L[1]*math.cos(th0+th1)
       + L[2]*math.cos(th0+th1+th2))
    phi = th0 + th1 + th2
    return [r, z, phi]

def jacobian(L, th0, th1, th2):
    r11 = L[0]*math.cos(th0) + L[1]*math.cos(th0+th1) + L[2]*math.cos(th0+th1+th2)
    r12 = L[1]*math.cos(th0+th1) + L[2]*math.cos(th0+th1+th2)
    r13 = L[2]*math.cos(th0+th1+th2)
    r21 = -(L[0]*math.sin(th0) + L[1]*math.sin(th0+th1) + L[2]*math.sin(th0+th1+th2))
    r22 = -(L[1]*math.sin(th0+th1) + L[2]*math.sin(th0+th1+th2))
    r23 = -L[2]*math.sin(th0+th1+th2)
    J = np.array([
        [r11, r12, r13],
        [r21, r22, r23],
        [1.0, 1.0, 1.0]
    ], dtype=np.float64)
    return J

def dls_step(L, target_rz, th, lam=1.0):
    cur = fk_rz(th[0], th[1], th[2], L)
    err = np.array([
        [target_rz[0] - cur[0]],
        [target_rz[1] - cur[1]],
        [angle_wrap(target_rz[2] - cur[2])]
    ], dtype=np.float64)
    J      = jacobian(L, th[0], th[1], th[2])
    JJt    = J @ J.T
    J_pinv = J.T @ np.linalg.inv(JJt + lam*np.eye(3))
    dth    = (J_pinv @ err).reshape(3)
    return dth, err, J

def ik_converges(L, target_rz, th0, max_iters=200):
    th  = th0.copy()
    lam = 1.0
    J   = jacobian(L, th[0], th[1], th[2])
    err = np.zeros((3, 1))
    for _ in range(max_iters):
        dth, err, J = dls_step(L, target_rz, th, lam)
        e   = float(err.T @ err)
        lam = e + 0.002
        th += dth
        if e < ERR_THRESH**2:
            U, S, Vt = np.linalg.svd(J, full_matrices=False)
            return True, e, S.min(), S.max()/S.min(), th
    U, S, Vt = np.linalg.svd(J, full_matrices=False)
    smin = S.min()
    cond = S.max()/S.min() if smin > 1e-8 else float('inf')
    return False, float(err.T @ err), smin, cond, th

def manipulability(J):
    det = np.linalg.det(J @ J.T)
    if det <= 1e-12:
        return -1e6
    return math.log(det)

def limit_margin_cost(th, limits=JOINT_LIMITS, eps=1e-3):
    s = 0.0
    for i, (lo, hi) in enumerate(limits):
        m = max(min(th[i]-lo, hi-th[i]), eps)
        s += math.log(m)
    return s

def merit_function(th, L):
    J = jacobian(L, th[0], th[1], th[2])
    return W_MANI*manipulability(J) + W_LIMIT*limit_margin_cost(th)

def finite_diff_grad(f, th, L, h=1e-3):
    g    = np.zeros(3, dtype=np.float64)
    base = f(th, L)
    for i in range(3):
        th_p    = th.copy()
        th_p[i] += h
        g[i]    = (f(th_p, L) - base) / h
    return g

def nullspace_projector(J):
    JJt    = J @ J.T
    J_pinv = J.T @ np.linalg.inv(JJt + 1e-6*np.eye(3))
    return np.eye(3) - J_pinv @ J

def cond_and_sigma(J):
    U, S, Vt = np.linalg.svd(J, full_matrices=False)
    smin = float(S.min())
    cond = float(S.max()/S.min()) if S.min() > 1e-8 else float('inf')
    return cond, smin

def e_task_norm(th, target_rz, L):
    cur  = fk_rz(th[0], th[1], th[2], L)
    dr   = target_rz[0] - cur[0]
    dz   = target_rz[1] - cur[1]
    dphi = angle_wrap(target_rz[2] - cur[2])
    return math.sqrt(dr*dr + dz*dz + dphi*dphi)

def joint_margins(th, limits):
    return [min(th[i]-lo, hi-th[i]) for i, (lo, hi) in enumerate(limits)]

# ---- プレグラスプ ----
def make_pregrasp_from_bottle(bxyz, approach_yaw_deg=YAW_LOCK_DEG, offset=PREGRASP_OFFSET):
    bx, by, bz = bxyz
    r     = math.sqrt(bx*bx + by*by)
    # ボトルのz(ベース基準) → IK用z_eff(J2基準)に変換
    z_eff = bz - L1_OFFSET
    yaw   = DegToRad(approach_yaw_deg)
    r_pre = max(r - offset, 0.02)
    return [r_pre, z_eff, yaw]

def merit_pregrasp_aug(th, L, th_pre=None):
    base    = merit_function(th, L)
    penalty = -5.0 * max(0.0, th[1])  # 肘上がりペナルティ
    bonus   =  1.0 * th[0]            # 肩前傾ボーナス
    return base + penalty + bonus

# ================== ROS2ノード ====================
class AmirTrajectoryNode(Node):
    def __init__(self):
        super().__init__('amir_trajectory_node')

        self.declare_parameter('L1', DEFAULT_L[0])
        self.declare_parameter('L2', DEFAULT_L[1])
        self.declare_parameter('L3', DEFAULT_L[2])
        self.declare_parameter('alpha',  0.3)   # IK収束速度（大きいほど速い）
        self.declare_parameter('k_null', K_NULL)
        # 速度コマンド [rad/s] ドライバのvelocity_buffer(300mrad/s)回避用
        self.declare_parameter('joint_vel', 0.05)  # 低速・安全設定
        # 単発モード: Trueなら1回コマンドを送って終了
        self.declare_parameter('one_shot', False)
        # Backward compatibility: mode が空なら use_sim から sim/real を決める。
        self.declare_parameter('use_sim', True)
        self.declare_parameter('mode', '')
        self.declare_parameter('real_unit', 'mrad')
        self.declare_parameter('real_output', 'joint_trajectory')
        self.declare_parameter('arm_trajectory_topic', '/arm_controller/joint_trajectory')
        self.declare_parameter('real_cmd_topic', '/motor_sub')
        self.declare_parameter('deadman_required', True)
        self.declare_parameter('deadman_topic', '/deadman_enabled')
        self.declare_parameter('palm_pose_timeout_sec', 0.3)
        self.declare_parameter('summary_log_period_sec', 1.0)
        self.declare_parameter('control_rate_hz', 30.0)

        self.L = [
            self.get_parameter('L1').value,
            self.get_parameter('L2').value,
            self.get_parameter('L3').value,
        ]
        self.alpha     = self.get_parameter('alpha').value
        self.k_null    = self.get_parameter('k_null').value
        self.joint_vel = self.get_parameter('joint_vel').value
        self.one_shot  = as_bool(self.get_parameter('one_shot').value)
        self.use_sim   = as_bool(self.get_parameter('use_sim').value)
        self.cmd_sent  = False  # 単発モード用フラグ

        requested_mode = str(self.get_parameter('mode').value).strip().lower()
        self.mode = requested_mode if requested_mode else ('sim' if self.use_sim else 'real')
        if self.mode not in ('sim', 'real', 'dry_run'):
            self.get_logger().warn(
                f'Invalid mode "{self.mode}". Falling back to dry_run for safety.')
            self.mode = 'dry_run'

        self.real_unit = str(self.get_parameter('real_unit').value).strip().lower()
        if self.real_unit not in ('mrad', 'rad'):
            self.get_logger().warn(
                f'Invalid real_unit "{self.real_unit}". Falling back to mrad for safety.')
            self.real_unit = 'mrad'

        self.real_output = str(self.get_parameter('real_output').value).strip().lower()
        if self.real_output not in ('joint_trajectory', 'amir_cmd'):
            self.get_logger().warn(
                f'Invalid real_output "{self.real_output}". Falling back to joint_trajectory.')
            self.real_output = 'joint_trajectory'

        self._arm_topic = str(self.get_parameter('arm_trajectory_topic').value)
        self._real_cmd_topic = str(self.get_parameter('real_cmd_topic').value)
        self.deadman_required = as_bool(self.get_parameter('deadman_required').value)
        self.deadman_topic = str(self.get_parameter('deadman_topic').value)
        self.palm_pose_timeout_sec = float(self.get_parameter('palm_pose_timeout_sec').value)
        self.summary_log_period_sec = float(self.get_parameter('summary_log_period_sec').value)
        self.control_rate_hz = max(float(self.get_parameter('control_rate_hz').value), 0.1)

        # 変更根拠: sim/real/dry_run を topic 名ではなく明示パラメータで分ける。
        # 期待される効果: 同じ PalmPose 入力を使いながら送信先と安全挙動を切り替えられる。
        if self.mode == 'sim':
            self._timer_period   = 0.5   # 0.5秒周期 (2Hz): Gazeboの制御応答に合わせる
            self._time_from_start_sec = 0  # 0秒 + nanosec指定で0.5秒
            self._time_from_start_nsec = 500_000_000  # 0.5秒
        elif self.mode == 'real':
            self._timer_period   = 2.0   # 2秒周期: 実機ドライバのvelocity_buffer対策
            self._time_from_start_sec = 2
            self._time_from_start_nsec = 0
        else:
            self._timer_period   = 1.0 / self.control_rate_hz
            self._time_from_start_sec = 0
            self._time_from_start_nsec = int(1_000_000_000 / self.control_rate_hz)

        self.get_logger().info(
            f'mode={self.mode} | real_unit={self.real_unit} | real_output={self.real_output} | '
            f'arm_topic={self._arm_topic} | real_cmd_topic={self._real_cmd_topic} | '
            f'deadman_required={self.deadman_required} | timer={self._timer_period}s | '
            f'control_rate_hz={self.control_rate_hz:.1f}'
        )

        # IK内部角の初期値（J2基準r-z平面）
        self.theta0 = 1.07   # J2相当（肩）
        self.theta1 = -0.77  # J3相当（肘）※下向きが負
        self.theta2 = 0.0    # J4相当（手首）

        # 状態変数
        self.palm_pose  = PoseStamped()
        self.bottle_xyz = np.zeros(3, dtype=np.float64)
        self.last_palm_time = None
        self.deadman_enabled = False
        self.palm_rx_count = 0
        self.last_palm_rx_time = None
        self.palm_interval_count = 0
        self.palm_interval_sum_sec = 0.0
        self.palm_interval_max_sec = 0.0
        self.command_tx_count = 0
        self.command_plan_count = 0
        self.last_summary_time = self.get_clock().now().nanoseconds * 1e-9
        self.last_summary_palm_count = 0
        self.last_summary_command_count = 0
        self.last_summary_command_plan_count = 0
        self.last_planned_rad = []
        self.last_planned_real = []
        self.last_ik_error = float('nan')
        self.last_ik_ok = False
        self.last_send_allowed = False

        # FSM
        self.phase           = Phase.TRACK
        self.last_switch_t   = 0.0
        self.realign_start_t = 0.0
        self.H_baseline      = None
        self.stand_rzp       = None

        # EMAフィルタ: 0.05で超低速追従（急変を防ぐ安全策）
        self.cmd_prev  = [0.0] * 5
        self.cmd_alpha = 0.05
        self.initialized = (self.mode == 'dry_run')  # dry_run は joint_states なしでも検証可能にする

        # Subscriber
        self.create_subscription(PoseStamped,      '/palm_pose',              self.palm_cb,         50)
        self.create_subscription(Float32MultiArray, '/identified_bottle_pose', self.bottle_cb,       10)
        self.create_subscription(JointState,        '/joint_states',           self.joint_states_cb, 10)
        self.create_subscription(Bool, self.deadman_topic, self.deadman_cb, 10)

        # Publisher
        self.arm_pub = None
        self.real_cmd_pub = None
        if self.mode == 'sim' or (self.mode == 'real' and self.real_output == 'joint_trajectory'):
            self.arm_pub = self.create_publisher(JointTrajectory, self._arm_topic, 10)
        if self.mode == 'real' and self.real_output == 'amir_cmd':
            self.real_cmd_pub = self.create_publisher(AmirCmd, self._real_cmd_topic, 10)
        self.metrics_pub = self.create_publisher(Float32MultiArray, '/amir_metrics',                   20)
        self.phase_pub   = self.create_publisher(String,            '/phase_name',                     10)

        # 制御タイマー: use_simで周期切替
        self.t0    = self.get_clock().now().nanoseconds * 1e-9
        self.timer = self.create_timer(self._timer_period, self.control_loop)
        self.get_logger().info(
            f'Amir Trajectory Node started  '
            f'[L={self.L}, L1_offset={L1_OFFSET}m, joint_vel={self.joint_vel}rad/s]'
        )

    # ---- コールバック ----
    def joint_states_cb(self, msg):
        name_to_idx = {n: i for i, n in enumerate(msg.name)}
        joint_map = {
            'Joint_1': 0, 'Joint_2': 1, 'Joint_3': 2,
            'Joint_4': 3, 'Joint_5': 4
        }
        for jname, cidx in joint_map.items():
            if jname in name_to_idx:
                self.cmd_prev[cidx] = msg.position[name_to_idx[jname]]

        # 起動時のみIK内部角を実機から初期化
        if not self.initialized:
            if 'Joint_2' in name_to_idx and 'Joint_3' in name_to_idx and 'Joint_4' in name_to_idx:
                j2 = msg.position[name_to_idx['Joint_2']]
                j3 = msg.position[name_to_idx['Joint_3']]
                j4 = msg.position[name_to_idx['Joint_4']]
                self.theta0 = float(np.clip(1.5708 - j2, JOINT_LIMITS[0][0], JOINT_LIMITS[0][1]))
                self.theta1 = float(np.clip(j3,           JOINT_LIMITS[1][0], JOINT_LIMITS[1][1]))
                self.theta2 = float(np.clip(j4,           JOINT_LIMITS[2][0], JOINT_LIMITS[2][1]))
            self.initialized = True
            self.get_logger().info(
                f'初期関節角を取得: {[f"{v:.3f}" for v in self.cmd_prev]}\n'
                f'IK初期値: th0={self.theta0:.3f} th1={self.theta1:.3f} th2={self.theta2:.3f}'
            )

    def palm_cb(self, msg):
        self.palm_pose = msg
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.last_palm_rx_time is not None:
            interval = max(now - self.last_palm_rx_time, 0.0)
            self.palm_interval_count += 1
            self.palm_interval_sum_sec += interval
            self.palm_interval_max_sec = max(self.palm_interval_max_sec, interval)
        self.last_palm_rx_time = now
        self.last_palm_time = now
        self.palm_rx_count += 1

    def deadman_cb(self, msg):
        self.deadman_enabled = bool(msg.data)

    def bottle_cb(self, msg):
        if len(msg.data) >= 3:
            self.bottle_xyz[0] = -float(msg.data[0])
            self.bottle_xyz[1] = -float(msg.data[2])
            self.bottle_xyz[2] =  float(msg.data[1])

    # ---- コマンド送信 ----
    def real_command_values(self, joint_angles_rad):
        """Convert internal rad commands to the selected real output unit.

        変更根拠: 単位変換を一箇所に集約し、IK 内部単位 rad と実機送信単位を分離する。
        期待される効果: mrad/rad の取り違えを dry_run ログで検出しやすくする。
        """
        angles = []
        velocities = []
        for i, angle_rad in enumerate(joint_angles_rad):
            if self.real_unit == 'mrad':
                # Mirrors amir_driver write(): command_rad * 1000 - initial_position[mrad].
                angle = float(angle_rad * 1000.0 - AMIR_DRIVER_INITIAL_POSITION_MRAD[i])
                delta = float(angle_rad - self.cmd_prev[i]) if i < len(self.cmd_prev) else 0.0
                velocity = math.copysign(self.joint_vel * 1000.0, delta) if abs(delta) > 1e-6 else 0.0
            else:
                # TODO: Confirm whether any real Amir driver variant expects raw rad in AmirCmd.
                angle = float(angle_rad)
                delta = float(angle_rad - self.cmd_prev[i]) if i < len(self.cmd_prev) else 0.0
                velocity = math.copysign(self.joint_vel, delta) if abs(delta) > 1e-6 else 0.0
            angles.append(angle)
            velocities.append(float(velocity))

        while len(angles) < 6:
            angles.append(0.0)
            velocities.append(0.0)
        return angles[:6], velocities[:6]

    def publish_joint_trajectory(self, joint_angles_rad):
        # velocities=0 + time_from_start: joint_trajectory_controller が補間する。
        msg = JointTrajectory()
        msg.joint_names = JOINT_NAMES
        point = JointTrajectoryPoint()
        point.positions       = [float(a) for a in joint_angles_rad]
        point.velocities      = [0.0] * len(JOINT_NAMES)  # controllerが自動補間
        point.time_from_start = Duration(
            sec=self._time_from_start_sec,
            nanosec=self._time_from_start_nsec
        )
        msg.points = [point]
        self.arm_pub.publish(msg)

    def publish_real_amir_cmd(self, joint_angles_rad):
        angles, velocities = self.real_command_values(joint_angles_rad)
        msg = AmirCmd()
        msg.angle = angles
        msg.vel = velocities
        self.real_cmd_pub.publish(msg)

    def publish_arm_command(self, joint_angles_rad):
        self.last_planned_rad = [float(a) for a in joint_angles_rad]
        self.last_planned_real, _ = self.real_command_values(joint_angles_rad)

        if self.mode == 'dry_run':
            return False
        if self.mode == 'sim':
            if self.arm_pub is None:
                self.get_logger().error('sim mode selected but JointTrajectory publisher is not available')
                return False
            self.publish_joint_trajectory(joint_angles_rad)
        elif self.real_output == 'joint_trajectory':
            if self.arm_pub is None:
                self.get_logger().error('real joint_trajectory output selected but publisher is not available')
                return False
            self.publish_joint_trajectory(joint_angles_rad)
        else:
            if self.real_cmd_pub is None:
                self.get_logger().error('real amir_cmd output selected but publisher is not available')
                return False
            self.publish_real_amir_cmd(joint_angles_rad)

        self.command_tx_count += 1
        return True

    def log_summary(self, now_abs, palm_fresh):
        if now_abs - self.last_summary_time < self.summary_log_period_sec:
            return
        dt = max(now_abs - self.last_summary_time, 1e-6)
        palm_hz = (self.palm_rx_count - self.last_summary_palm_count) / dt
        command_plan_hz = (self.command_plan_count - self.last_summary_command_plan_count) / dt
        command_tx_hz = (self.command_tx_count - self.last_summary_command_count) / dt
        command_hz = command_plan_hz if self.mode == 'dry_run' else command_tx_hz
        if self.palm_interval_count > 0:
            avg_interval_ms = (self.palm_interval_sum_sec / self.palm_interval_count) * 1000.0
            max_interval_ms = self.palm_interval_max_sec * 1000.0
        else:
            avg_interval_ms = float('nan')
            max_interval_ms = float('nan')
        ik_state = 'ok' if self.last_ik_ok else 'fail'
        self.get_logger().info(
            'summary '
            f'mode={self.mode} real_unit={self.real_unit} '
            f'palm_hz={palm_hz:.2f} palm_avg_interval_ms={avg_interval_ms:.1f} '
            f'palm_max_interval_ms={max_interval_ms:.1f} '
            f'cmd_hz={command_hz:.2f} cmd_tx_hz={command_tx_hz:.2f} '
            f'deadman={self.deadman_enabled} palm_fresh={palm_fresh} '
            f'send_allowed={self.last_send_allowed} '
            f'ik={ik_state} ik_err={self.last_ik_error:.4f} '
            f'planned_rad={[round(v, 4) for v in self.last_planned_rad]} '
            f'planned_real={[round(v, 2) for v in self.last_planned_real]}'
        )
        self.last_summary_time = now_abs
        self.last_summary_palm_count = self.palm_rx_count
        self.last_summary_command_count = self.command_tx_count
        self.last_summary_command_plan_count = self.command_plan_count
        self.palm_interval_count = 0
        self.palm_interval_sum_sec = 0.0
        self.palm_interval_max_sec = 0.0

    # ---- メイン制御ループ ----
    def control_loop(self):
        now_abs = self.get_clock().now().nanoseconds * 1e-9
        # 実機の初期関節角が取得できるまで待機（急な動き防止）
        if not self.initialized:
            self.get_logger().info('joint_states待機中...', throttle_duration_sec=1.0)
            self.log_summary(now_abs, palm_fresh=False)
            return
        now = now_abs - self.t0

        palm_fresh = (
            self.last_palm_time is not None and
            (now_abs - self.last_palm_time) <= self.palm_pose_timeout_sec
        )
        deadman_ok = (not self.deadman_required) or self.deadman_enabled

        # 変更根拠: PalmPose が途絶えた状態の古い姿勢追従は実機で危険。
        # 期待される効果: timeout 時は新規指令を送らず、状態だけを1Hzで確認できる。
        if not palm_fresh:
            self.last_send_allowed = False
            self.log_summary(now_abs, palm_fresh=False)
            return

        # palm_pose取得（ベース座標系, z=0がAmirの足元）
        px = self.palm_pose.pose.position.x
        py = self.palm_pose.pose.position.y
        pz = self.palm_pose.pose.position.z

        # ---- J1: 水平旋回 ----
        j1_target = math.atan2(py, px)

        # ---- r-z平面IK用ターゲット ----
        r_target  = math.sqrt(px**2 + py**2)
        z_eff     = pz - L1_OFFSET          # ベース基準z → J2基準z_eff
        phi_target = DegToRad(YAW_LOCK_DEG)
        palm_rzp   = [r_target, z_eff, phi_target]

        # ---- プレグラスプ & ブレンド ----
        if np.linalg.norm(self.bottle_xyz) > 1e-6:
            pre_rzp = make_pregrasp_from_bottle(self.bottle_xyz)
            dist    = math.sqrt(
                (px - self.bottle_xyz[0])**2 +
                (py - self.bottle_xyz[1])**2)
            beta = 0.0
        else:
            pre_rzp = palm_rzp
            beta    = 0.0
            dist    = r_target

        if (APPROACH_LOCK_BETA and
                self.phase == Phase.APPROACH and
                np.linalg.norm(self.bottle_xyz) > 1e-6):
            beta = 1.0

        target_rzp = [
            (1-beta)*palm_rzp[0] + beta*pre_rzp[0],
            (1-beta)*palm_rzp[1] + beta*pre_rzp[1],
            (1-beta)*palm_rzp[2] + beta*pre_rzp[2],
        ]
        if self.phase == Phase.BACKOFF and self.stand_rzp is not None:
            target_rzp = self.stand_rzp

        # プレグラスプ関節角 θ_pre
        th_pre = None
        if np.linalg.norm(self.bottle_xyz) > 1e-6:
            ok_pg, _, _, _, th_try = ik_converges(
                self.L, pre_rzp,
                np.array([self.theta0, self.theta1, self.theta2]))
            if ok_pg:
                th_pre = th_try

        # ---- 主タスク: DLS-IK (r, z_eff のみ追跡) ----
        lam = 1.0
        for _ in range(150):
            cur    = fk_rz(self.theta0, self.theta1, self.theta2, self.L)
            err_r  = target_rzp[0] - cur[0]
            err_z  = target_rzp[1] - cur[1]
            e      = err_r**2 + err_z**2
            lam    = e + 0.002
            J      = jacobian(self.L, self.theta0, self.theta1, self.theta2)
            J_rz   = J[:2, :]
            J_pinv = J_rz.T @ np.linalg.inv(J_rz @ J_rz.T + lam*np.eye(2))
            dth    = (J_pinv @ np.array([[err_r], [err_z]])).reshape(3)
            self.theta0 += float(self.alpha * dth[0])
            self.theta1 += float(self.alpha * dth[1])
            self.theta2 += float(self.alpha * dth[2])
            if e < 1e-6:
                break

        # ---- ヌル空間最適化 ----
        th_vec   = np.array([self.theta0, self.theta1, self.theta2])
        J_now    = jacobian(self.L, self.theta0, self.theta1, self.theta2)
        cond_now, sigma_min_now = cond_and_sigma(J_now)
        mani_now = manipulability(J_now)
        m0, m1, m2 = joint_margins(th_vec, JOINT_LIMITS)
        H_before = merit_pregrasp_aug(th_vec, self.L, th_pre)
        e_before = e_task_norm(th_vec, target_rzp, self.L)

        P        = nullspace_projector(J_now)
        gradH    = finite_diff_grad(
            lambda th, L_: merit_pregrasp_aug(th, L_, th_pre),
            th_vec, self.L)
        dth_null = self.k_null * (P @ gradH)
        self.theta0 += float(dth_null[0])
        self.theta1 += float(dth_null[1])
        self.theta2 += float(dth_null[2])

        # 関節リミットクリップ
        self.theta0 = float(np.clip(self.theta0, JOINT_LIMITS[0][0], JOINT_LIMITS[0][1]))
        self.theta1 = float(np.clip(self.theta1, JOINT_LIMITS[1][0], JOINT_LIMITS[1][1]))
        self.theta2 = float(np.clip(self.theta2, JOINT_LIMITS[2][0], JOINT_LIMITS[2][1]))

        # 評価 (後)
        th_vec_after = np.array([self.theta0, self.theta1, self.theta2])
        H_after      = merit_pregrasp_aug(th_vec_after, self.L, th_pre)
        dH_null      = H_after - H_before
        e_after      = e_task_norm(th_vec_after, target_rzp, self.L)
        ik_position_error = math.hypot(
            target_rzp[0] - fk_rz(self.theta0, self.theta1, self.theta2, self.L)[0],
            target_rzp[1] - fk_rz(self.theta0, self.theta1, self.theta2, self.L)[1])
        self.last_ik_error = float(ik_position_error)
        self.last_ik_ok = bool(np.isfinite(ik_position_error) and ik_position_error < 0.05)
        Jdnull       = J_now @ dth_null.reshape(3, 1)
        Jdnull_norm  = float(np.linalg.norm(Jdnull))
        dnull_norm   = float(np.linalg.norm(dth_null))
        de_due_null  = e_after - e_before

        # FK検証（デバッグ用）
        self.last_ee_position = fk_3d(j1_target, self.theta0, self.theta1, self.theta2, self.L)

        # ---- FSM 遷移 ----
        cooldown_ok = (now - self.last_switch_t) > CD_SWITCH
        near        = (np.linalg.norm(self.bottle_xyz) > 1e-6 and
                       dist >= 0.0 and dist <= D_IN + 0.03)
        m_min       = min(m0, m1, m2)
        bad_posture = (sigma_min_now < SIGMA_MIN_THR or
                       cond_now > COND_MAX_THR or
                       m_min < MARGIN_MIN_THR)
        Phase_name  = self.phase.name

        if self.phase == Phase.TRACK:
            if cooldown_ok and near and bad_posture:
                r_stand        = pre_rzp[0] + BACKOFF_D
                self.stand_rzp = [r_stand, pre_rzp[1], pre_rzp[2]]
                self.last_switch_t = now
                Phase_name     = "BACKOFF"

        elif self.phase == Phase.BACKOFF:
            cur_rzp = fk_rz(self.theta0, self.theta1, self.theta2, self.L)
            err_bo  = math.hypot(
                self.stand_rzp[0]-cur_rzp[0],
                self.stand_rzp[1]-cur_rzp[1])
            if err_bo < 0.01:
                self.phase           = Phase.REALIGN
                self.realign_start_t = now
                self.H_baseline      = H_after
                self.last_switch_t   = now
                Phase_name           = "REALIGN"

        elif self.phase == Phase.REALIGN:
            improved    = ((H_after - self.H_baseline) > IMPROVE_H_MIN
                           if self.H_baseline else False)
            timeout     = (now - self.realign_start_t) > REALIGN_DT_MAX
            criteria_ok = (sigma_min_now >= SIGMA_MIN_THR*1.2 and
                           cond_now <= COND_MAX_THR*0.8 and
                           m_min >= MARGIN_MIN_THR)
            if cooldown_ok and (improved or timeout or criteria_ok):
                self.phase         = Phase.APPROACH
                self.last_switch_t = now
                Phase_name         = "APPROACH"

        elif self.phase == Phase.APPROACH:
            cur_rzp = fk_rz(self.theta0, self.theta1, self.theta2, self.L)
            err_ap  = math.hypot(
                target_rzp[0]-cur_rzp[0],
                target_rzp[1]-cur_rzp[1])
            if err_ap < 0.01:
                self.phase         = Phase.TRACK
                self.last_switch_t = now
                Phase_name         = "TRACK"

        # ---- Amirの5DOF関節角に変換 ----
        # Joint_1: 水平旋回
        j1_cmd = float(np.clip(j1_target + 2.50, -2.96706, 2.96706))
        # TODO: +2.50オフセットはAmirの実機ゼロ点に合わせて実測調整が必要

        # Joint_2: IK theta0 → Amir J2角度
        # Amirのゼロ点=水平、IKのゼロ点=垂直 → π/2オフセット必要
        joint_2 = float(np.clip(1.5708 - self.theta0, 0.0, 2.356194))

        # Joint_3: IK theta1 → Amir J3角度（ゼロ点一致）
        joint_3 = float(np.clip(self.theta1, -2.792527, 0.0))

        # Joint_4: IK theta2 → Amir J4角度
        joint_4 = float(np.clip(self.theta2, -2.094395, 1.308997))

        # Joint_5: 手首ロール（固定）
        joint_5 = 0.0

        joint_angles_rad = [j1_cmd, joint_2, joint_3, joint_4, joint_5]

        # ---- Publish ----
        self.last_planned_rad = [float(a) for a in joint_angles_rad]
        self.last_planned_real, _ = self.real_command_values(joint_angles_rad)
        self.last_send_allowed = bool(deadman_ok and palm_fresh)

        # 変更根拠: Deadman OFF でゼロ姿勢や停止姿勢を送ると、意図せず実機を動かす可能性がある。
        # 期待される効果: OFF/timeout 時は新規指令を止め、最後の状態保持は下位 controller/driver に任せる。
        if self.last_send_allowed:
            self.command_plan_count += 1
            self.publish_arm_command(joint_angles_rad)

        phase_msg      = String()
        phase_msg.data = Phase_name
        self.phase_pub.publish(phase_msg)

        metrics_msg      = Float32MultiArray()
        metrics_msg.data = [
            float(now), float(dist), float(0.0), float(e_after),
            float(sigma_min_now), float(cond_now),
            float(mani_now), float(m0), float(m1), float(m2),
            float(H_after), float(dH_null),
            float(Jdnull_norm), float(dnull_norm),
            float(de_due_null), float(self.phase.value)
        ]
        self.metrics_pub.publish(metrics_msg)

        self.log_summary(now_abs, palm_fresh=palm_fresh)


def main(args=None):
    rclpy.init(args=args)
    node = AmirTrajectoryNode()
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
