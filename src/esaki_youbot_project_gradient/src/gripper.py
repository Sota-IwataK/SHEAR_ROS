#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Amirグリッパ制御ノード（ROS2 Humble）
youbot用gripper.pyをROS2/Amirに最適化
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String, Bool
from sensor_msgs.msg import JointState

# サービス/メッセージ定義のインポート
# 実機環境に応じて適切なパッケージ名を指定してください
try:
    from dynamixel_workbench_msgs.srv import DynamixelCommand, GripperCmd
except ImportError:
    # 実行時にエラーを出すのではなく、警告として処理（開発環境用）
    print("Warning: dynamixel_workbench_msgs not found. Service calls will fail.")

# ================== 定数 =================================
MOTOR_ID_0 = 0
MOTOR_ID_1 = 1
SET_TORQUE = 75
TORQUE_THRESHOLD = 50.0  # N・m (または単位系に準ずる)

# トピック・サービス名
SRV_DYNAMIXEL_CMD = '/dynamixel_workbench/dynamixel_command'
SRV_GRIPPER_EXEC  = '/dynamixel_workbench/execution'
TOPIC_JOINT_STATE = '/dynamixel_workbench/joint_states'
TOPIC_GRASP_CMD   = '/grasp_command'
TOPIC_GRIPPER_STATE = '/gripper_state'

# ================== グリッパノード =======================

class AmirGripperNode(Node):
    def __init__(self):
        super().__init__('amir_gripper')

        # ROS2では並行処理を許可するためにCallback Groupを使用
        self.callback_group = ReentrantCallbackGroup()

        self.is_open = False
        self.joint_names = []

        # --- Publisher ---
        self.state_pub = self.create_publisher(Bool, TOPIC_GRIPPER_STATE, 10)

        # --- Subscriber ---
        # サービス呼び出しを伴うコールバックにはcallback_groupを指定
        self.grasp_cmd_sub = self.create_subscription(
            String,
            TOPIC_GRASP_CMD,
            self.grasp_cb,
            10,
            callback_group=self.callback_group
        )
        self.joint_state_sub = self.create_subscription(
            JointState,
            TOPIC_JOINT_STATE,
            self.torque_cb,
            10
        )

        # --- Service Client ---
        self.dynamixel_cmd_client = self.create_client(
            DynamixelCommand, 
            SRV_DYNAMIXEL_CMD,
            callback_group=self.callback_group
        )
        self.gripper_exec_client = self.create_client(
            GripperCmd, 
            SRV_GRIPPER_EXEC,
            callback_group=self.callback_group
        )

        # 初期化処理を別メソッドとして実行
        self._init_gripper()

    def _init_gripper(self):
        """初期化ルーチン"""
        if not self.dynamixel_cmd_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Service not available: DynamixelCommand')
            return
        
        self.get_logger().info('Initializing Amir Gripper...')
        # トルク設定
        self._send_dynamixel_command(MOTOR_ID_0, 'Goal_Current', SET_TORQUE)
        self._send_dynamixel_command(MOTOR_ID_1, 'Goal_Current', SET_TORQUE)
        
        # 初期状態で閉じる
        self._send_gripper_exec('close')
        self.is_open = False
        self.get_logger().info('AmirGripperNode Initialized')

    def _send_dynamixel_command(self, motor_id: int, addr_name: str, value: int):
        """DynamixelCommandを呼び出す（同期風に非同期処理を待機）"""
        req = DynamixelCommand.Request()
        req.command = 'command'
        req.id = motor_id
        req.addr_name = addr_name
        req.value = value

        # 非同期で呼び出し、完了まで待機
        future = self.dynamixel_cmd_client.call_async(req)
        # 注意: Node内部のメソッドでrclpy.spin_until...は使用せず、
        # MultiThreadedExecutor環境下でfuture.result()を待つ形式を推奨
        return future

    def _send_gripper_exec(self, command: str):
        """GripperCmdを呼び出す"""
        req = GripperCmd.Request()
        req.command = command
        future = self.gripper_exec_client.call_async(req)
        return future

    # ---- コールバック ----

    async def grasp_cb(self, msg: String):
        """把持コマンド受信：awaitを使用してブロッキングを回避"""
        command = msg.data
        self.get_logger().info(f'Received command: {command}')

        if command == 'close' and self.is_open:
            await self._send_gripper_exec('close')
            self.is_open = False
        elif command == 'open' and not self.is_open:
            await self._send_gripper_exec('open')
            self.is_open = True
        else:
            self.get_logger().debug('Command ignored or already in state.')

    def torque_cb(self, msg: JointState):
        """トルク監視：把持判定の計算"""
        # インデックス指定ではなく、名前ベースでの取得を推奨（ロバスト性向上）
        # ※ 実機の名前(joint_names)が不明なため、今回は平均値のロジックを継承
        if len(msg.effort) < 2:
            return

        # 把持判定の数理モデル: $\tau_{avg} = \frac{|\tau_0| + |\tau_1|}{2}$
        m0_torque = abs(msg.effort[0])
        m1_torque = abs(msg.effort[1])
        avg_torque = (m0_torque + m1_torque) / 2.0

        object_grasped = avg_torque > TORQUE_THRESHOLD

        state_msg = Bool()
        state_msg.data = bool(object_grasped)
        self.state_pub.publish(state_msg)

# ================== エントリーポイント ===================

def main(args=None):
    rclpy.init(args=args)
    node = AmirGripperNode()

    # MultiThreadedExecutorを使用してデッドロックを防止
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
