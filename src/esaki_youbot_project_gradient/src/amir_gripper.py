#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String, Bool
from sensor_msgs.msg import JointState
from control_msgs.action import GripperCommand

GRIPPER_OPEN_POS  = 0.5
GRIPPER_CLOSE_POS = 0.0
MAX_EFFORT        = 10.0
TORQUE_THRESHOLD  = 50.0
ACTION_GRIPPER    = '/gripper_controller/gripper_cmd'

class AmirGripperNode(Node):
    def __init__(self):
        super().__init__('amir_gripper')
        self.is_open = False
        self._action_client = ActionClient(self, GripperCommand, ACTION_GRIPPER)
        self.state_pub = self.create_publisher(Bool, '/gripper_state', 10)
        self.create_subscription(String, '/grasp_command', self.grasp_cb, 10)
        self.create_subscription(JointState, '/joint_states', self.joint_state_cb, 10)
        self.get_logger().info('AmirGripperNode started')

    def send_gripper_goal(self, position):
        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = MAX_EFFORT
        self._action_client.wait_for_server(timeout_sec=5.0)
        self._action_client.send_goal_async(goal)

    def grasp_cb(self, msg):
        cmd = msg.data.lower().strip()
        self.get_logger().info(f'Received: {cmd}')
        if cmd == 'open':
            self.send_gripper_goal(GRIPPER_OPEN_POS)
            self.is_open = True
        elif cmd == 'close':
            self.send_gripper_goal(GRIPPER_CLOSE_POS)
            self.is_open = False

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
