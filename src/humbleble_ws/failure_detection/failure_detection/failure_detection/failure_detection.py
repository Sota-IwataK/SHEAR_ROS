
from .bayesian_network import generate_bn
from .bayesian_network import MyNode

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.callback_groups import ReentrantCallbackGroup    #   並列にコールバック実行可能

import numpy as np
from bt_generator.node_template import *    #   テンプレートはbt_generatorと共通
from behavior_tree_msgs.msg import BTStatus
from behavior_tree_msgs.msg import NodeStatus
from std_msgs.msg import String
# from observation_msgs.msg import PathEvaluation
from nav_msgs.msg import Odometry
from failure_detection_msgs.srv import PathCheck
from failure_detection_msgs.srv import BeforePick
from failure_detection_msgs.srv import AfterPick
from failure_detection_msgs.msg import Solution
from observation_msgs.msg import PathEvaluation
from nav_msgs.msg import OccupancyGrid
from my_nav_msgs.msg import PathInfo
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState
from behavior_tree_msgs.srv import SetBlackBoard
from behavior_tree_msgs.msg import BBPose
from geometry_msgs.msg import PoseStamped

import copy

import time
from bt_generator.bt_generator_base import BTGeneratorBase
# class NavObservation(Node):
#     def __init__(self):
#         super().__init__('nav_observation')

        

    



class FailureDetection(Node):

    def __init__(self):
        super().__init__('failure_Detection')
        self.pose_dict = {}
        self.pick_target_pose = PoseStamped()

         
        # pose = PoseStamped()
        # solution_msg = Solution()  
        # self.refine_movement_of_viewpoint(pose, solution_msg)
        # print(solution_msg)

        #   BayesianNetworkの生成
        self.bn_dict = {}

        self.bn_dict["undetected_error"] = generate_bn.BN("undetected_error")
        # self.bn_dict["drop_obj"] = generate_bn.BN("drop_obj")
        self.global_observation_dict = {}  #   リアルタイムな共通の観測

        self.arm1_angle = [0] * 5 
        self.arm2_angle = [0] * 5


        # self.bn_dict["path_check"]= generate_bn.BN("")
        # Multithreading 
        self.sub_group = MutuallyExclusiveCallbackGroup()
        self.service_group = ReentrantCallbackGroup()

        self.grasping = False

        
        self.srv = self.create_service(PathCheck, 'path_check', self.path_check_cb, callback_group=self.service_group)
        self.before_pick_server = self.create_service(BeforePick, 'before_pick', self.before_pick_cb, callback_group=self.service_group)
        self.after_pick_server = self.create_service(AfterPick, 'after_pick', self.after_pick_cb, callback_group=self.service_group)


        self.global_costmap = OccupancyGrid()
        self.local_costmap = OccupancyGrid()
        self.map = OccupancyGrid()

        #   publisher
        self.solution_publisher = self.create_publisher(Solution, 'bt_solution', 10)

        #   subscriber
        self.joint_state_sub = self.create_subscription(JointState,'/joint_states', self.joint_state_cb, 10, callback_group=self.sub_group)
        self.robot_pose_sub = self.create_subscription(Odometry,'tf_odom', self.tf_odom_cb, 10, callback_group=self.sub_group)
        self.gripper_state_sub = self.create_subscription(Bool,'gripper_state', self.gripper_state_cb, 10, callback_group=self.sub_group)
        self.global_cost_sub = self.create_subscription(OccupancyGrid, 'global_costmap/costmap', self.global_cost_cb,10, callback_group=self.sub_group)
        self.local_cost_sub = self.create_subscription(OccupancyGrid, 'local_costmap/costmap', self.local_cost_cb,10, callback_group=self.sub_group)
        self.map_sub = self.create_subscription(OccupancyGrid, 'map', self.map_sub_cb,10, callback_group=self.sub_group)



    def gripper_state_cb(self, msg):
        self.grasping = msg.data

    def joint_state_cb(self, msg):
        if msg.name[0] == "arm_joint_1":
            self.arm1_angle[0] = -msg.position[0]+np.deg2rad(169)
            self.arm1_angle[1] = msg.position[1]-np.deg2rad(65)
            self.arm1_angle[2] = msg.position[2]+np.deg2rad(146)
            self.arm1_angle[3] = msg.position[3]-np.deg2rad(102.5)
            self.arm1_angle[4] = -msg.position[4]+np.deg2rad(167.5)
        elif msg.name[0] == "arm_2_joint_1":
            self.arm2_angle[0] = -msg.position[0]+np.deg2rad(169)
            self.arm2_angle[1] = -msg.position[1]+np.deg2rad(65)
            self.arm2_angle[2] = -msg.position[2]-np.deg2rad(146)
            self.arm2_angle[3] = -msg.position[3]+np.deg2rad(102.5)
            self.arm2_angle[4] = -msg.position[4]+np.deg2rad(167.5)
        
        if abs(self.arm2_angle[0]) < np.deg2rad(20):
            self.global_observation_dict["arm_2_joint_0"] = "abs<20deg"
            # print("abs<20deg")
        else:
            self.global_observation_dict["arm_2_joint_0"] = "other"
            # print("other")


    def before_pick_cb(self, request, response):
        print("before_pick_cb")
        result = self.bn_dict["undetected_error"].predict_proba({})
        #   目標位置を保存しておく
        self.pick_target_pose = request.pose
        #   立ってるか、倒れているか
        if request.pose.pose.orientation.w == 1:
            self.global_observation_dict["target_shape"] = "vertically_long"
        else:
            self.global_observation_dict["target_shape"] = "horizontally_long"

        # self.bn_dict["drop_obj"].print_all_probability(result)

        return response

    def after_pick_cb(self, request, response):
        print("--------after_pick_cb------------")
        print("bt_node_name = ", request.bt_node_name)
        print("pick_error_code = ", request.error_string)
        observation_dict = {}
        error_name = None

        skip = False

        if request.error_string == "Search_target_obj_failed" or "approach_obj_failed":
            error_name = "undetected_error"
            observation_dict["undetected_error"] = "error"
            skip = True
        # else:
        #     observation_dict["img_undetected_error"] = "no_error" #   

        # #   approach中に見失った（最初が誤認識だった説もある）
        # if request.error_string == "approach_obj_failed":
        #     print("ssss")

        if not skip:
            #  grasp_result
            if request.error_string == "grasp_failed":
                error_name = "grasp_result"
                observation_dict["pick_failure"]=  "failure"
                skip = True
            else:
                observation_dict["pick_failure"]=  "not_failure"

        if not skip:
            #  drop_obj
            if request.error_string == "drop":
                error_name = "drop_obj"
                observation_dict["pick_failure"] = "failure"
                skip = True
            else:
                observation_dict["pick_failure"] = "not_failure"

        if not skip:
            #   正常終了
            if request.error_string == "success":
                error_name = "pickResult"
                return response
            else:
                print("??????????????")
        

        observation_dict.update(self.global_observation_dict)   #   共通の観測と結合
        
        #   確率計算
        result = self.bn_dict["undetected_error"].predict_proba(observation_dict)
        #   確率が高い原因を抽出
        all_cause_list = self.bn_dict["undetected_error"].get_all_cause_dict(error_name, result)
        print("all_cause_list",all_cause_list)

        #   各原因に対応する解決策の名前を取得
        solutions = self.bn_dict["undetected_error"].get_solutions(all_cause_list)
        
        # self.bn_dict["grasp_result"].print_all_probability(result)

        solution_msg = Solution()   #   メッセージ
        for solution in solutions:
            #   視点移動の場合
            if solution == "movement_of_viewpoint":
                self.refine_movement_of_viewpoint(self.pick_target_pose, solution_msg)  #   リカバリ条件
            elif solution == "Spin":
                solution_msg.solutions.append(actions["Spin"].xml)    #   リカバリアクション

        # print("solutions",solution_msg.solutions)

        #   解決策があればpublishする
        if len(solution_msg.solutions) > 0:
            solution_msg.bt_node_name = request.bt_node_name   #   アクションサブツリー名
            self.solution_publisher.publish(solution_msg)

        return response
   

    def tf_odom_cb(self, msg):
        value = msg.twist.twist.linear.x * msg.twist.twist.linear.x + msg.twist.twist.linear.y * msg.twist.twist.linear.y
    #     if value  > 10 * 10:
    #         self.observe_dict["base_pose_trip"] = "True"

    def global_cost_cb(self, msg):
        # print("global_cost_cb")
        self.global_costmap = msg

    def local_cost_cb(self, msg):
        # print("local_cost_cb")
        self.local_costmap = msg

    def map_sub_cb(self, msg):
        self.map = msg
        


    # def learn_cb(self, msg):
    #     print("learn_cb",msg)

    # def path_cost_cb(self, msg):
    #     print(msg)
    #     if msg.max_cost > 50:
    #         self.observe_dict["close_to_obj"] = "local_cost>90"
    #     # else:
    #     #     self.observe_dict["close_to_obj"] = "local_cost<=90"
    
   

    def path_check_cb(self, request, response):
        print("path_check_cb")
        #   BN作成
        # if "path_check" not in self.bn_dict:
        #     self.bn_dict["path_check"]= generate_bn.BN()
        #     self.bn_dict["path_check"].generate_bn("FollowPath")

        max_cost = 0
        ave_cost = 0
        for i, path_with_condition in enumerate(request.paths):
            for pose in path_with_condition.path.poses:
                
                value = self.get_cost_from_position(self.global_costmap, pose.pose.position)
                ave_cost +=value
                if max_cost < value:
                    max_cost = value

            # print(request.path.poses[-1].pose.position)
            # front_map_value = self.get_cost_from_position(self.map, request.path.poses[0].pose.position)
            # back_map_value = self.get_cost_from_position(self.map, request.path.poses[-1].pose.position)
            
            # print("front_map_value", front_map_value)
            # print("back_map_value", back_map_value)

            print("max_cost=",max_cost)

            path_info = PathInfo()
            if max_cost>=80:

                # success_rate
                path_info.success_rate = -1.0
                response.info_set.append(path_info)

        return response

    
    def get_cost_from_position(self, map, position):
        if map.info.resolution==0:
            print("map is not received")
            return -1

        x_grid = position.x / map.info.resolution
        y_grid = position.y / map.info.resolution
        origin_x_grid = map.info.origin.position.x / map.info.resolution
        origin_y_grid = map.info.origin.position.y / map.info.resolution
        dx = int(x_grid - origin_x_grid)
        dy = int(y_grid - origin_y_grid)

        if dx < 0 or dy < 0:
            print("out of cost")
            return -1

        index = int(dy * map.info.width + dx)

        if len(map.data) > index:
            return map.data[index]
        else:
            print("out of cost2")
            return -1
            





    #  コールバック内の処理時間が長いと詰まるので注意
    def bt_status_callback(self, msg):
        # print(msg)
        
        #アクションノードが実行中の場合、BNを生成する
        # if msg.node_type == msg.ACTION and msg.status != msg.IDLE:

        #   とりあえずBTのルートが失敗したときだけ分析する
        if msg.root_status.status != NodeStatus.FAILURE:
            return


        #   全ノードに対して実行
        for bt_node_status in msg.bt_status:
            if bt_node_status.node_type == NodeStatus.ACTION and bt_node_status.status == NodeStatus.FAILURE:
                # print(bt_node_status)
                #BNのノードに登録していないアクションは無視する
                if bt_node_status.node_name not in generate_bn.AllNodeSet.action_mapping: 
                    continue

                #アクションに対応するBNを生成していない場合、生成する
                if (bt_node_status.node_name, bt_node_status.node_id) not in self.bn_dict:
                    model = generate_bn.BN()
                    model.generate_bn(bt_node_status.node_name)
                    self.bn_dict[(bt_node_status.node_name, bt_node_status.node_id)] = model
                    print("generte")
                

                
                if bt_node_status.status != bt_node_status.RUNNING :
                    #成功か失敗した場合
                    print(bt_node_status.status)
                    print(bt_node_status.node_name)
                    node_status =  generate_bn.AllNodeSet.action_mapping[bt_node_status.node_name][bt_node_status.status]# BNの状態変数名を取得

                    status_dict = {bt_node_status.node_name:node_status}
                    status_dict = dict(**status_dict, **self.observe_dict)
                    # print("node_status",node_status)
                    result = self.bn_dict[(bt_node_status.node_name, bt_node_status.node_id)].model.predict_proba(status_dict)# アクションノードの結果を観測した場合の確率計算

                    # self.bn_dict[(bt_node_status.node_name, bt_node_status.node_id)].print_all_probability(result)# 観測後
                else:
                    #実行中の場合
                    # print(bt_node_status.status)
                    result = self.bn_dict[bt_node_status.node_name, bt_node_status.node_id].model.predict_proba(self.observe_dict)# 観測していない場合の確率計算

                    # self.bn_dict[(bt_node_status.node_name, bt_node_status.node_id)].print_probability(result, bt_node_status.node_name)# 観測前


                
                self.bn_dict[(bt_node_status.node_name, bt_node_status.node_id)].print_all_probability(result)# 全ノードの事後確率表示

        #   グラフの表示を続ける（コールバックの外に書くべき）
        for bn_node in self.bn_dict.values():
            bn_node.graph_show()
            
            

    #   視点移動の位置候補を返す
    def refine_movement_of_viewpoint(self, target_pose, solution_msg):
        angle = 0
        dist = 0.7  #   物体からの半径
        while angle < 2*np.pi and rclpy.ok():
            bb_pose = BBPose()
            
            goal_candidate = copy.deepcopy(target_pose)
            goal_candidate.pose.position.x += dist * np.cos(angle)
            goal_candidate.pose.position.y += dist * np.sin(angle) 

            #   blackboardに候補地点を書き込む
            bb_pose.key = "temp_pose_" + "{:.1f}".format(goal_candidate.pose.position.x) + "," + "{:.1f}".format(goal_candidate.pose.position.y)
            bb_pose.value = goal_candidate
            solution_msg.bb_message.poses.append(bb_pose)

            #   xmlにキーを設定
            c = etree.fromstring(constraints["IsRobotCloseTo"].xml)
            c.attrib['location_id'] = bb_pose.key
            c.attrib['location_pose'] = "{" + bb_pose.key + "}"
            c.attrib['inner_radius'] = "0.0"
            c.attrib['outer_radius'] = "0.3"    #   その地点上にいること条件としているので許容誤差は小さく
            c.attrib['position_only'] = "true"
            condition_string = etree.tostring(c, encoding="unicode", pretty_print=True)
            solution_msg.solutions.append(condition_string)

            angle += np.pi/2    #   解像度
        return 





        # print(msg)
        # self.get_logger().info('I heard: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    fd_node = FailureDetection()

    rclpy.spin(fd_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    fd_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()