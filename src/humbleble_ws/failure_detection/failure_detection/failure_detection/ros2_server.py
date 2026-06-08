import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from failure_detection_msgs.action import Analysys


class FailureDetectionServer(Node):

    def __init__(self):
        super().__init__('failure_detection_server')
        self._action_server = ActionServer(
            self,
            Analysys,
            'analysys',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        # self.get_logger().info('Executing goal...')
        self.get_logger().info('Executing goal...')
        
        print(goal_handle.request.node_name)
        result = Analysys.Result()
        result.success_probability = 1.0
        goal_handle.succeed()#アクションの状態を保存
        return result


def main(args=None):
    rclpy.init(args=args)

    failure_detection_server = FailureDetectionServer()

    rclpy.spin(failure_detection_server)


if __name__ == '__main__':
    main()