import sys
import os
from typing import no_type_check_decorator
from lxml.builder import E
from lxml import etree
import copy
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action import ActionClient

import numpy as np
from behavior_tree_msgs.srv import SetBlackBoard
from behavior_tree_msgs.srv import GetBT
from behavior_tree_msgs.action import GenApproach
from behavior_tree_msgs.msg import BTStatus
from behavior_tree_msgs.msg import NodeStatus
from behavior_tree_msgs.msg import BBPath
from behavior_tree_msgs.action import ExecuteTree
from failure_detection_msgs.srv import PathCheck
from .node_template import *
from .bt_generator_base import *
from nav2_msgs.action import ComputePathToPose
from threading import Event
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup    #   並列にコールバック実行可能
from my_nav_msgs.msg import PathWithCondition
from my_nav_msgs.srv import SetObjCost
from observation_msgs.srv import GetObjects

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


"-----------------------------------------------------------------------------------------------------"       

class BTNavGeneratorService(BTGeneratorBase):
    def __init__(self, name="bt_nav_gen"):
        super().__init__(name) #   super()で親の初期化に追記できる
        
        self.paths_with_condition = []  #   生成した経路候補
        self.goal_pose = None
        self._compute_path_to_pose_client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose') #   経路計画アクションクライアント
        #   目標を受信するアクションサーバー
        self._action_server = ActionServer(self,GenApproach, name, self.generate_cb, cancel_callback=self.cancel_callback)


        #   経路確認
        self.path_check_client = self.create_client(PathCheck, 'path_check')    #   経路確認（アクションにしたほうがいいと思う）
        timer_period = 1  # 秒
        self.timer_cb_group = ReentrantCallbackGroup()
        self.timer = self.create_timer(timer_period, self.timer_callback, callback_group=self.timer_cb_group)   

        #   コストマップ設定
        self.get_obj_client = self.create_client(GetObjects, 'get_objects')    #
        self.set_obj_cost_client = self.create_client(SetObjCost, 'set_obj_cost')    #
        


    

    #   BT生成アクションコールバック
    def generate_cb(self, goal_handle):
        self.get_logger().info('Executing goal!!!!...')
        self.create_bt_request()    #   bt_executorにアクション通信を生成してもらうサービスリクエスト

        self.goal_pose = goal_handle.request.pose
        # self.paths_with_condition.clear()
        # path_with_condition = PathWithCondition()
        # path_with_condition.path = self.compute_path(self.goal_pose)
        # self.paths_with_condition.append(path_with_condition)   #   経路候補追加
        self.compute_multi_path(self.goal_pose)



        self.execute_tree(self.createActionTree())

        goal_handle.succeed()   #   正常終了報告
        result = GenApproach.Result()

        print("candidates",len(self.paths_with_condition))
        if len(self.paths_with_condition) == 0:
            result.error_string = "failure"
        else:
            result.error_string = "success"

        return result


    #   タイマーコールバック（経路チェックなど）
    def timer_callback(self):
        # self.set_param("planner_server", [self.get_param_msg("GridBased.allow_unknown", False)])
        # self.set_param("planner_server/set_parameters", 
        #                 [self.get_param_msg("GridBased.allow_unknown", False),
        #                 self.get_param_msg("GridBased.use_astar", True),
        #                 ])
        if len(self.paths_with_condition) == 0:
            return
        self.path_check()


    def set_obj_cost(self, command):
        get_obj_req = GetObjects.Request()
        get_obj_req.command = GetObjects.Request.ALL_COLLISION
        get_obj_res = self.service_request(self.get_obj_client, get_obj_req)
        get_obj_res = GetObjects.Response()
        # marker_array = MarkerArray()
        set_cost_req = SetObjCost.Request()
        
        for obj in get_obj_res.objects:
            marker = Marker()
            marker.header.frame_id = "map"
            marker.pose = obj.pose
            marker.type = Marker.CYLINDER
            marker.scale.x = 0.1
            marker.text = obj.id
            if command == "ADD":
                marker.action = Marker.ADD
            elif command == "DELETE":
                marker.action = Marker.DELETE
            
            set_cost_req.markers.append(marker)
    
        self.service_request(self.set_obj_cost_client, set_cost_req)
                    

        


    def path_check(self):
        req = PathCheck.Request()
        req.paths = self.paths_with_condition
        res = self.service_request(self.path_check_client, req)

        if res is not None:
            update_bb = False

            for i, path_info in enumerate(res.info_set):
                if path_info.success_rate == -1:
                    update_bb = True
                    self.get_logger().info('path_check: not ok')
                    

                    # self.paths_with_condition.clear()
                    path_with_condition = PathWithCondition()

                    goal_pose = self.paths_with_condition[i].path.poses[-1] #   元の経路の目標位置を抽出
                    goal_pose.header = self.paths_with_condition[i].path.header #   ヘッダーも抽出
              
                    #   経路再計画
                    path_with_condition.path = self.compute_path(goal_pose)
                    self.paths_with_condition[i] = path_with_condition  #   経路更新

                    # self.paths_with_condition.append(path_with_condition)   
                    
                    
                    # self.execute_tree(self.createActionTree())
                    # self.compute_path(goal_handle.request.pose)
                else:
                    self.get_logger().info('path_check: ok')
            if update_bb == True:
                self.bb_req = SetBlackBoard.Request()
                self.set_bb_path()
                self.set_black_board()
    

    #   複数の接近の仕方を考える（よさげな方法を優先して計画したい）
    #   接近位置、姿勢等
    def compute_multi_path(self, goal_pose):
        self.paths_with_condition.clear()
        angle = 0
        dist = 0.5
        while angle < 2*np.pi and rclpy.ok():
            goal_candidate = copy.deepcopy(goal_pose)
            goal_candidate.pose.position.x += dist * np.cos(angle)
            goal_candidate.pose.position.y += dist * np.sin(angle) 

            path_with_condition = PathWithCondition()
            path_with_condition.path = self.compute_path(goal_candidate)
            self.paths_with_condition.append(path_with_condition)   #   経路候補追加
            angle += np.pi/6


    def compute_path(self, goal_pose):
        #   経路計画
        goal_msg = ComputePathToPose.Goal()
        goal_msg.goal = goal_pose
        result = self.action_send(self._compute_path_to_pose_client, goal_msg)
        
        return result.path


        


    def createActionTree(self):
        self.bb_req = SetBlackBoard.Request()   #   BB初期化
        root = etree.Element("root")
        root.attrib['main_tree_to_execute']="MainTree"
        bt = etree.Element("BehaviorTree")
        bt.attrib['ID']="MainTree"

        #   優先度付きフォールバック
        # pfallback = etree.Element("PriorityFallback")
        # pfallback.attrib['priority']="{priority}"

        pfallback = etree.Element("Fallback")

        for i, path in enumerate(self.paths_with_condition):
            action_xml = etree.fromstring(follow_path_xml)
            action_xml.attrib['path'] = "{path" + str(i) + "}"
            pfallback.append(action_xml)
            bb_path = BBPath()
            bb_path.key = "path" + str(i)   #   キー：path1, path2, ...
            bb_path.path = path.path
            self.bb_req.paths.append(bb_path)
        bt.append(pfallback)
        root.append(bt)
        tree = etree.tostring(root, encoding="unicode", pretty_print=True)
        return tree


    def set_bb_path(self):
        for i, path in enumerate(self.paths_with_condition):
            bb_path = BBPath()
            bb_path.key = "path" + str(i)   #   キー：path1, path2, ...
            bb_path.path = path.path
            self.bb_req.paths.append(bb_path)



class CreateGeneratorClass:
    def __init__(self):
        self.node_dict = {}
        node = Node("bt_nav_gen0")
        self.executor = MultiThreadedExecutor()  #   これじゃないと止まってしまう
        self.srv = node.create_service(CreateBT, 'create_nav_gen', self.callback)

        

        self.executor.add_node(node)
        # self.executor.remove_node()
        self.executor.spin()

    def callback(self, request, response):
        
        if request.command=="add":
            print("add", request.bt_name)
            self.node_dict[request.bt_name] = BTNavGeneratorService(request.bt_name)
            self.executor.add_node(self.node_dict[request.bt_name])
        elif request.command=="remove":
            print("remove", request.bt_name)
            self.executor.remove_node(self.node_dict[request.bt_name])
            removed_node = self.node_dict.pop(request.bt_name)

            #   TODO:ライフサイクルを導入して改善できる？
            if removed_node.destroy_node():
                print("success")
            else:
                print("failed")
            
            
            # 
        return response
    


def main():
    rclpy.init()
    CreateGeneratorClass()
    # bt_generator = BTNavGeneratorService("bt1")
    # bt_generator2 = BTNavGeneratorService("bt2")

    # executor = MultiThreadedExecutor()  #   これじゃないと止まってしまう

    # executor.add_node(bt_generator)
    # executor.add_node(bt_generator2)
    
    # executor.spin()
    # # rclpy.spin(bt_generator, executor)

    # bt_generator.destroy_node()
    # rclpy.shutdown()



if __name__ == '__main__':
    main()

