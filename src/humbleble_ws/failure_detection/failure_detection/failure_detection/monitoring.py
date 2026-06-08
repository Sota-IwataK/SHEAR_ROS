
from .bayesian_network import generate_bn
from .bayesian_network import MyNode

import rclpy
from rclpy.node import Node

from behavior_tree_msgs.msg import BTStatus
from behavior_tree_msgs.msg import NodeStatus
from std_msgs.msg import String
from observation_msgs.msg import PathEvaluation
from nav_msgs.msg import Odometry
from observation_msgs.srv import PathCheck


class BTSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.bt_sub = self.create_subscription(
            BTStatus,
            'bt_status',
            self.bt_status_callback,
            10)

        self.path_cost_sub = self.create_subscription(
            PathEvaluation,
            'path_evaluation',
            self.path_cost_cb,
            10)

        self.robot_psoe_sub = self.create_subscription(
            Odometry,
            'tf_odom',
            self.tf_odom_cb,
            10)

            

        self.learn_sub = self.create_subscription(
            String,
            'learn_msg',
            self.learn_cb,
            10)

            
        self.bt_sub  # prevent unused variable warning
        self.observe_dict = {}  # 観測したノードと状態変数名（キー：ノード名、値：状態変数名）
        self.bn_dict = {}  # キー：（ノード名、ノードid）、値：BN
        self.observe_dict["base_pose_trip"] = "False"
        self.observe_dict["close_to_obj"] = "local_cost<=90"


    def learn_cb(self, msg):
        print("learn_cb",msg)

    def path_cost_cb(self, msg):
        print(msg)
        if msg.max_cost > 50:
            self.observe_dict["close_to_obj"] = "local_cost>90"
        # else:
        #     self.observe_dict["close_to_obj"] = "local_cost<=90"
    
    def tf_odom_cb(self, msg):
        value = msg.twist.twist.linear.x * msg.twist.twist.linear.x + msg.twist.twist.linear.y * msg.twist.twist.linear.y
        if value  > 10 * 10:
            self.observe_dict["base_pose_trip"] = "True"




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
            
            
                
                
                   
                    
                



        # print(msg)
        # self.get_logger().info('I heard: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    bt_subscriber = BTSubscriber()

    rclpy.spin(bt_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    bt_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()