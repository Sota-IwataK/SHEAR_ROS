import sys
import os
from typing import no_type_check_decorator
from lxml.builder import E
from lxml import etree
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action import ActionClient
from rclpy.action import CancelResponse
import numpy as np
from behavior_tree_msgs.srv import SetBlackBoard
from behavior_tree_msgs.srv import GetBT
from behavior_tree_msgs.action import GenApproach
from behavior_tree_msgs.msg import BTStatus
from behavior_tree_msgs.msg import NodeStatus
from behavior_tree_msgs.msg import BBPath
from behavior_tree_msgs.action import ExecuteTree

from .node_template import *
from nav2_msgs.action import ComputePathToPose
from threading import Event
from rclpy.executors import MultiThreadedExecutor



def get_xml(self):
        dirname = os.path.dirname(__file__)
        parent_dir = '/'.join(dirname.split('/')[:-1]) 
        xml_dir = parent_dir + '/bt_xml/navigation.xml'

        parser = etree.XMLParser(remove_blank_text=True, recover=True, remove_comments=True)
        with open(xml_dir, mode='rt', encoding='utf-8') as f:
            tree = etree.parse(f, parser=parser)
        # print(etree.tostring(tree).decode())
        root = tree.getroot()
        return root





class BTGeneratorService(Node):
    def __init__(self):
        super().__init__('minimal_client_async')
        self.paths = []  #   生成した経路候補
        self.bb_req = SetBlackBoard.Request()
        self.action_done_event = Event()
        self.execute_tree_done_event = Event()
        self._action_client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose') #   経路計画アクションクライアント
        self._execute_tree_client = ActionClient(self, ExecuteTree, 'execute_bt') #   ツリーをexecuterに送信するアクションクライアント
        self.send_tree_req = GetBT.Request()
        # self.client = self.create_client(GetBT, 'bt_update')    #   ツリーをexecuterに送信するサービスクライアント
        
        self.bb_client = self.create_client(SetBlackBoard, 'set_blackboard')
        while not self.bb_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        
        


        #   目標を受信するアクションサーバー
        self._action_server = ActionServer(self,GenApproach,
            'gen_approach',self.generate_cb, cancel_callback=self.cancel_callback)

    def cancel_callback(self, cancel_request):
        future = self.execute_tree_goal_handle.cancel_goal_async()
        future.add_done_callback(self.goal_canceled_callback)

        return CancelResponse.ACCEPT

    def goal_canceled_callback(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('Cancelling of goal complete')
        else:
            self.get_logger().warning('Goal failed to cancel')
            


            
    def set_black_board(self):
        self.bb_future = self.bb_client.call_async(self.bb_req)
        while rclpy.ok():
            if self.bb_future.done():
                try:
                    response = self.bb_future.result()
                except Exception as e:
                    self.get_logger().info(
                        'Service call failed %r' % (e,))
                else:
                    print("set_black_board: ok")
                break
                

    def send_tree(self, bt):
        print(bt)
        self.set_black_board()  #   behavior treeに変数を登録
        self.send_tree_req.bt = bt
        # self.req.bt = get_xml()
        self.future = self.client.call_async(self.send_tree_req)

        if self.future.done():
            try:
                response = self.future.result()
            except Exception as e:
                self.get_logger().info(
                    'Service call failed %r' % (e,))

    def execute_tree(self, bt):
        print(bt)
        self.set_black_board()  #   behavior treeに変数を登録
        goal_msg = ExecuteTree.Goal()
        goal_msg.bt = bt
        self._execute_tree_client.wait_for_server()

        self.execute_tree_done_event.clear()
        self._execute_tree_future = self._execute_tree_client.send_goal_async(goal_msg)
        self._execute_tree_future.add_done_callback(self.execute_tree_response_callback)

        # Wait for action to be done
        self.execute_tree_done_event.wait()


    def createActionTree(self):
        root = etree.Element("root")
        root.attrib['main_tree_to_execute']="MainTree"
        bt = etree.Element("BehaviorTree")
        bt.attrib['ID']="MainTree"

        fallback = etree.Element("Fallback")
        for i, path in enumerate(self.paths):
            action_xml = etree.fromstring(follow_path_xml)
            action_xml.attrib['path'] = "{path" + str(i) + "}"
            fallback.append(action_xml)
            bb_path = BBPath()
            bb_path.key = "path" + str(i)   #   キー：path1, path2, ...
            bb_path.path = path
            self.bb_req.paths.append(bb_path)
        bt.append(fallback)
        root.append(bt)
        tree = etree.tostring(root, encoding="unicode", pretty_print=True)
        return tree


    #   BT生成アクションコールバック
    def generate_cb(self, goal_handle):
        self.get_logger().info('Executing goal...')
        self.compute_path(goal_handle.request.pose)
        self.execute_tree(self.createActionTree())


        goal_handle.succeed()   #   正常終了報告
        result = GenApproach.Result()

        print("candidates",len(self.paths))
        if len(self.paths) == 0:
            result.error_string = "failure"
        else:
            result.error_string = "success"

        return result

    def compute_path(self, goal_pose):
        self._sendgoal = True
        #   経路計画
        goal_msg = ComputePathToPose.Goal()
        goal_msg.goal = goal_pose
        self._action_client.wait_for_server()

        self.action_done_event.clear()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

        # Wait for action to be done
        self.action_done_event.wait()

    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        # print(result.path)
        self.paths.append(result.path)   #   経路候補追加
        self.action_done_event.set()
        # self.get_logger().info('Result: {0}'.format(result.path))


    def execute_tree_response_callback(self, future):
        self.execute_tree_goal_handle = future.result()
        if not self.execute_tree_goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._execute_tree_result_future = self.execute_tree_goal_handle.get_result_async()
        self._execute_tree_result_future.add_done_callback(self.execute_tree_result_callback)

    def execute_tree_result_callback(self, future):
        result = future.result().result
        # print(result.path)
        # self.paths.append(result.path)   #   経路候補追加
        self.execute_tree_done_event.set()
        # self.get_logger().info('Result: {0}'.format(result.path))
        


def main():
    rclpy.init()
    bt_generator = BTGeneratorService()

    executor = MultiThreadedExecutor()  #   これじゃないと止まってしまう

    rclpy.spin(bt_generator, executor)

    bt_generator.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()

