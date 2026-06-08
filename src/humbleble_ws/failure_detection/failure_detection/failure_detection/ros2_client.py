import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from failure_detection_msgs.action import Analysys


class AnalysysActionClient(Node):

    def __init__(self):
        super().__init__('analysys_action_client')
        self._action_client = ActionClient(self, Analysys, 'analysys')

    def send_goal(self, order):
        goal_msg = Analysys.Goal()
        goal_msg.node_id = order
        goal_msg.node_name = "test"
        self._action_client.wait_for_server()

        return self._action_client.send_goal_async(goal_msg)


def main(args=None):
    rclpy.init(args=args)

    action_client = AnalysysActionClient()

    future = action_client.send_goal(10)

    rclpy.spin_until_future_complete(action_client, future)


if __name__ == '__main__':
    main()