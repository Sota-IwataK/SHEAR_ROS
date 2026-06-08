#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String, Bool
from sensor_msgs.msg import JointState
from control_msgs.action import GripperCommand

TORQUE_THRESHOLD  = 50.0
ACTION_GRIPPER    = '/gripper_controller/gripper_cmd'

class AmirGripperNode(Node):
    def __init__(self):
        super().__init__('amir_gripper')
        self.declare_parameter('open_position', -1.0)
        self.declare_parameter('close_position', 0.06)
        self.declare_parameter('max_effort', 10.0)

        self.open_position = float(self.get_parameter('open_position').value)
        self.close_position = float(self.get_parameter('close_position').value)
        self.max_effort = float(self.get_parameter('max_effort').value)

        self.is_open = False
        self._action_client = ActionClient(self, GripperCommand, ACTION_GRIPPER)
        self.state_pub = self.create_publisher(Bool, '/gripper_state', 10)
        self.create_subscription(String, '/grasp_command', self.grasp_cb, 10)
        self.create_subscription(JointState, '/joint_states', self.joint_state_cb, 10)
        self.get_logger().info('AmirGripperNode started')

    def send_gripper_goal(self, position):
        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = self.max_effort
        self._action_client.wait_for_server(timeout_sec=5.0)
        self._action_client.send_goal_async(goal)

    def grasp_cb(self, msg):
        cmd = msg.data.lower().strip()
        self.get_logger().info(f'Received: {cmd}')
        if cmd == 'open':
            self.send_gripper_goal(self.open_position)
            self.is_open = True
        elif cmd == 'close':
            self.send_gripper_goal(self.close_position)
            self.is_open = False
        else:
            self.get_logger().warn(f'Invalid grasp command: {msg.data}')

    def joint_state_cb(self, msg):
        if 'Gripper' not in msg.name:
            return
        idx = msg.name.index('Gripper')
        if idx < len(msg.effort):
            grasped = abs(msg.effort[idx]) > TORQUE_THRESHOLD
            self.state_pub.publish(Bool(data=grasped))

def main(args=None):
    rclpy.init(args=args)
    node = AmirGripperNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
