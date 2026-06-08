from pomegranate import *
import itertools
from behavior_tree_msgs.msg import NodeStatus
from bt_generator.node_template import *    #   テンプレートはbt_generatorと共通



# node_dict = {}
# action_mapping = {}

# observable_node_list = []



class NodeSet:
    def __init__(self):
        self.node_dict = {}
        self.cause_list = []   #   generate_OrCPTで生成したノード
        self.solution_dict = {} #   BNノードに対応する解決策　
        self.action_mapping = {}
        self.observable_node_list = []
        self.edge_list = []
        self.initialize()

        

    def initialize(self):
        "---------------認識周り----------------"
        self.node_dict["recognition_position_error"] = DiscreteDistribution({"small_error": 0.5, "medium_error": 0.4, "big_error": 0.1}) #   認識誤差

        self.node_dict["img_undetected_error_by_yolo"] = DiscreteDistribution({"True": 0.2, "False": 0.8}) #   認識

        
        
        
        # self.node_dict["target_pose"] = #   観測
        self.node_dict["cluster_of_same_obj"] = DiscreteDistribution({"cluster": 0.5, "not_cluster": 0.5}) #   観測
        
        
        self.node_dict["obj_occlusion"] = DiscreteDistribution({"True": 0.5, "False": 0.5}) # 観測


          
        self.node_dict["target_not_exist_by_img_false_detection"] = DiscreteDistribution({"True": 0.3, "False": 0.7})   #   情報源が過去の認識以外の場合（指示された）はFalseにする
        
        self.node_dict["arm_2_joint_0"] = DiscreteDistribution({"abs<20deg": 0.5, "other": 0.5}) # 観測
        # self.node_dict["target_location"] = DiscreteDistribution({"FR": 0.3, "FL": 0.7, "F": 1/8, ""})
        ##########Pick動作関連###########################################################



        ##########Place動作関連###########################################################
        self.node_dict["target_weight"] = DiscreteDistribution({"light": 1/3, "normal": 1/3, "heavy": 1/3})
        self.node_dict["place_error"] = DiscreteDistribution({"under2": 0.3, "other": 0.7})
        
        self.node_dict["far_from_center"] = DiscreteDistribution({"far": 0.5, "near": 0.5}) #   観測
        self.node_dict["target_shape"] = DiscreteDistribution({"vertically_long": 0.5, "horizontally_long": 0.5})  #   観測（ボトルの場合standingならvertically_long）


        # self.node_dict["grasp_shallow"] = DiscreteDistribution({"shallow": 0.3, "not_shallow": 0.7})
        
        
        self.node_dict["close_to_obj"] = DiscreteDistribution({"local_cost>90": 0.5, "local_cost<=90": 0.5})#障害物が近い確率
        
        self.node_dict["bad_footing"] = DiscreteDistribution({"bad": 0.1, "not_bad": 0.9})#足場が悪い確率
        self.node_dict["path_not_found_by_no_path"] = DiscreteDistribution({"True": 0.5, "False": 0.5})#真に完全に経路が存在しない場合
        self.node_dict["fewer_features"] = DiscreteDistribution({"True": 0.5, "False": 0.5})    #   環境の特徴が少ない（廊下みたいなやつ）


        self.update()
        self.set_edge()




    #   CPTの初期化
    def update(self):    
        "--------------アーム関連----------------"
        self.node_dict["arm_collision"] = DiscreteDistribution({"collision": 0.2, "no_collision": 0.8}) 


        self.node_dict["invalid_location"] =  DiscreteDistribution({"invalid": 0.0, "valid": 1.0})  #   指定された場所が異なる

        self.node_dict["target_not_exist_by_invalid_location"] = ConditionalProbabilityTable(
            [["invalid", "True", 0.9],
            ["invalid", "False", 0.1],
            ["valid", "True", 0.0],
            ["valid", "False", 1.0]],
            [self.node_dict["invalid_location"]])

        self.node_dict["target_not_exist_by_arm_collision"] = ConditionalProbabilityTable(
            [["collision", "True", 0.9],
            ["collision", "False", 0.1],
            ["no_collision", "True", 0.0],
            ["no_collision", "False", 1.0]],
            [self.node_dict["arm_collision"]])




        "---------------オブジェクト関連----------------" 
        self.node_dict["target_exist"] = self.generate_OrCPT(["target_not_exist_by_arm_collision", "target_not_exist_by_invalid_location", "target_not_exist_by_img_false_detection"], ["exist", "not_exist"])
 

        "---------------認識関連----------------"
        self.node_dict["self_occlusion"] = ConditionalProbabilityTable(    #   
            [["abs<20deg", "True", 0.9],
            ["abs<20deg", "False", 0.1],
            ["other", "True", 0.0],    
            ["other", "False", 1.0]],
            [self.node_dict["arm_2_joint_0"]])
        
        self.solution_dict["self_occlusion"] = actions["Spin"].xml  #   リカバリアクション


        self.node_dict["occlusion"] = self.generate_OrCPT(["self_occlusion", "obj_occlusion"], ["non_occlusion", "occlusion"])

        self.node_dict["recognition_orientation_error"] = ConditionalProbabilityTable(    #   
            [["vertically_long", "error", 0.0],
            ["vertically_long", "no_error", 1.0],
            ["horizontally_long", "error", 0.4],    #   倒れてる場合に姿勢誤差が出やすい
            ["horizontally_long", "no_error", 0.6]],
            [self.node_dict["target_shape"]])


        # self.node_dict["img_undetected_error_by_yolo"] = ConditionalProbabilityTable( #   YOLOの精度が原因で検出されなかった確率
        #     [["not_exist", "True", 0.0],    #   objが存在しなくても検出する場合（誤検出）もあるので1ではない。ややこしい
        #     ["not_exist", "False", 1.0],
        #     ["exist", "True", 0.25],
        #     ["exist", "False", 0.75]],
        #     [self.node_dict["target_exist"]])    

        self.node_dict["img_undetected_error_by_target_not_exist"] = ConditionalProbabilityTable( #   目標が存在しないことが原因で検出されなかった確率
            [["not_exist", "True", 0.7],    #   objが存在しなくても検出する場合（誤検出）もあるので1ではない。ややこしい
            ["not_exist", "False", 0.3],
            ["exist", "True", 0.0],
            ["exist", "False", 1.0]],
            [self.node_dict["target_exist"]])
        


        self.node_dict["img_undetected_error_by_occlusion"] = ConditionalProbabilityTable(    #   オクルージョンが原因で検出されなかった確率
            [["occlusion", "True", 0.9],
            ["occlusion", "False", 0.1],
            ["non_occlusion", "True", 0.0],
            ["non_occlusion", "False", 1.0]],
            [self.node_dict["occlusion"]])

        self.node_dict["img_undetected_error"] = self.generate_OrCPT(["img_undetected_error_by_yolo", 
                                                                "img_undetected_error_by_occlusion", 
                                                                "img_undetected_error_by_target_not_exist"], ["no_error", "error"])


        
        # self.node_dict["obj_not_detect_by_img_undetected_error"] = ConditionalProbabilityTable(    #   オクルージョンが原因で検出されなかった確率
        #     [["error", "True", 1.0],
        #     ["error", "False", 0.0],
        #     ["no_error", "True", 0.0],
        #     ["no_error", "False", 1.0]],
        #     [self.node_dict["img_undetected_error"]])

        

        


        # self.node_dict["obj_not_detect"] = self.generate_OrCPT([self.node_dict["obj_not_detect_by_img_undetected_error"], self.node_dict["obj_not_detect_by_target_not_exist"]], ["detect", "not_detect"])



        # "duplicate_detection" 重複検出


        
        

        # self.node_dict["img_recognition"] = ConditionalProbabilityTable(
        #     [["not_exist", "detect", 0.3],
        #     ["not_exist", "undetect", 0.7],
        #     ["exist", "detect", 0.75],
        #     ["exist", "undetect", 0.25]],
        #     [self.node_dict["target_exist"]])
        


        
        # "unintentional_move_obj"


        "---------------Pick動作関連----------------"
        self.node_dict["grasp_failed_by_recognition_orientation_error"] = ConditionalProbabilityTable(
            [["error", "True", 0.9],
            ["error", "False", 0.1],
            ["no_error", "True", 0.0],
            ["no_error", "False", 1.0]],
            [self.node_dict["recognition_orientation_error"]])


# small_error": 0.5, "medium_error": 0.4, "big_error"
        self.node_dict["grasp_failed_by_recognition_position_error"] = ConditionalProbabilityTable(
            [["big_error", "True", 1.0],
            ["big_error", "False", 0.0],
            ["medium_error", "True", 0.2],
            ["medium_error", "False", 0.8],
            ["small_error", "True", 0.0],
            ["small_error", "False", 1.0]],
            [self.node_dict["recognition_position_error"]])

        self.node_dict["grasp_failed_by_target_not_exist"] = ConditionalProbabilityTable(
            [["not_exist", "True", 1.0],
            ["not_exist", "False", 0.0],
            ["exist", "True", 0.0],
            ["exist", "False", 1.0]],
            [self.node_dict["target_exist"]])


        self.node_dict["grasp_result"]= self.generate_OrCPT(["grasp_failed_by_recognition_orientation_error", 
                                                        "grasp_failed_by_recognition_position_error", 
                                                        "grasp_failed_by_target_not_exist"], ["success", "failure"])    #ロボットが衝突する確率




        self.node_dict["grasp_shallow_by_recognition_position_error"] = ConditionalProbabilityTable(
            [["big_error", "True", 0.0],    #   誤差が大きすぎると把持すらできないので
            ["big_error", "False", 1.0],
            ["medium_error", "True", 0.8],
            ["medium_error", "False", 0.2],
            ["small_error", "True", 0.0],
            ["small_error", "False", 1.0]],
            [self.node_dict["recognition_position_error"]])


        self.node_dict["grasp_shallow"]= self.generate_OrCPT(["grasp_shallow_by_recognition_position_error"], ["not_shallow", "shallow"])    



        self.node_dict["grasp_slip_by_grasp_shallow"] = ConditionalProbabilityTable(
            [["shallow", "True", 0.6],
            ["shallow", "False", 0.4],
            ["not_shallow", "True", 0.0],
            ["not_shallow", "False", 1.0]],
            [self.node_dict["grasp_shallow"]])



        #   xy平面において物体の重心から離れていることが原因
        self.node_dict["grasp_slip_by_far_from_center"] = ConditionalProbabilityTable(
            [["far", "True", 0.9],   #   縦長
            ["far", "False", 0.1],
            ["near", "True", 0.0],
            ["near", "False", 1.0]],
            [self.node_dict["far_from_center"]])


        #   objの形状による（横長（倒れてる）、縦長（立ってる））
        # self.node_dict["grasp_slip_by_far_from_center"] = ConditionalProbabilityTable(
        #     [["far", "vertically_long", "True", 0.5],   #   縦長
        #     ["far", "vertically_long", "False", 0.5],
        #     ["far", "horizontally_long", "True", 0.9],  #   横長
        #     ["far", "horizontally_long", "False", 0.1],
        #     ["near", "vertically_long","True", 0.0],   
        #     ["near", "vertically_long","False", 1.0],
        #     ["near", "horizontally_long", "True", 0.0],
        #     ["near", "horizontally_long", "False", 1.0]],
        #     [self.node_dict["far_from_center"], self.node_dict["target_shape"]])

        self.node_dict["grasp_slip"] = self.generate_OrCPT(["grasp_slip_by_grasp_shallow", "grasp_slip_by_far_from_center"], ["not_slip", "slip"])


        self.node_dict["bound_by_weight"] = ConditionalProbabilityTable(
            [["light", "True", 0.5],
                ["light", "False", 0.5],
                ["normal", "True", 0.1],
                ["normal", "False", 0.9],
                ["heavy", "True", 0.1],
                ["heavy", "False", 0.9]],
            [self.node_dict["target_weight"]])
  

        self.node_dict["bound_by_error"] = ConditionalProbabilityTable(
            [["under2", "True", 0.1],
            ["under2", "False", 0.9],
            ["other", "True", 0.7],
            ["other", "False", 0.3]],
            [self.node_dict["place_error"]])


        self.node_dict["bound_obj"]=self.generate_OrCPT(["bound_by_weight","bound_by_error"], ["not_bound", "bound"])    #ロボットが衝突する確率
    
        
     

        #バウンドする原因が1つでもTrueなら必ずバウンドもTrueにする
        self.node_dict["lying_by_bound"] = ConditionalProbabilityTable(
        [["bound", "True", 0.8],
            ["bound", "False", 0.2],
            ["not_bound", "True", 0],
            ["not_bound", "False", 1]],
            [self.node_dict["bound_obj"]])


        self.node_dict["lying_by_grasp_slip"] = ConditionalProbabilityTable(
        [["slip", "True", 0.8],
            ["slip", "False", 0.2],
            ["not_slip", "True", 0],
            ["not_slip", "False", 1]],
            [self.node_dict["grasp_slip"]])
            

        
        # self.generate_OrCPT([self.node_dict["bound_by_weight"],self.node_dict["bound_by_error"]], ["not_crash", "crash"])    #ロボットが衝突する確率


        self.node_dict["place_result"]=self.generate_OrCPT(["lying_by_bound","lying_by_grasp_slip"], ["standing", "lying"])    

        #   重さが原因で落とす確率
        self.node_dict["drop_by_weight"] = ConditionalProbabilityTable(
            [["light", "True", 0.0],
                ["light", "False", 1.0],
                ["normal", "True", 0.5],
                ["normal", "False", 0.5],
                ["heavy", "True", 1.0],
                ["heavy", "False", 0.0]],
            [self.node_dict["target_weight"]])

        
        self.node_dict["drop_by_grasp_shallow"] = ConditionalProbabilityTable(
        [["shallow", "True", 0.7],
            ["shallow", "False", 0.3],
            ["not_shallow", "True", 0],
            ["not_shallow", "False", 1]],
            [self.node_dict["grasp_shallow"]])

        #   滑って落とした確率
        self.node_dict["drop_by_grasp_slip"] = ConditionalProbabilityTable(
        [["slip", "True", 0.6],
            ["slip", "False", 0.4],
            ["not_slip", "True", 0],
            ["not_slip", "False", 1]],
            [self.node_dict["grasp_slip"]])

        #   dropは把持状態からopenしてないのに状態遷移した場合と定義する
        self.node_dict["drop_obj"]=self.generate_OrCPT(["drop_by_weight","drop_by_grasp_shallow","drop_by_grasp_slip"], ["not_drop", "drop"])    #ロボットが衝突する確率


        # 

      
        ################################################################
        ##########移動動作関連###########################################
        

        

        self.node_dict["base_control_error_by_bad_footing"] = ConditionalProbabilityTable(#足場が悪い場合に制御異常が出る確率
            [["bad", "True", 0.8],
            ["bad", "False", 0.2],
            ["not_bad", "True", 0],
            ["not_bad", "False", 1]],
            [self.node_dict["bad_footing"]])
        

        self.node_dict["base_control_error"] = self.generate_OrCPT(["base_control_error_by_bad_footing"],["no_error", "error"])#ベースの制御異常が出る確率 
      



        self.node_dict["base_crash_by_base_control_error"] = ConditionalProbabilityTable(#制御異常で衝突する確率（何もない場所ではぶつからないと思うがとりあえず）
            [["error", "True", 1/2],
            ["error", "False", 1/2],
            ["no_error", "True", 0],
            ["no_error", "False", 1]],
            [self.node_dict["base_control_error"]])
 

        self.node_dict["base_crash_by_close_to_obj"] = ConditionalProbabilityTable(#障害物に近すぎて衝突する確率
            [["local_cost>90", "True", 0.7],
            ["local_cost>90", "False", 0.3],
            ["local_cost<=90", "True", 0.1],
            ["local_cost<=90", "False", 0.9]],
            [self.node_dict["close_to_obj"]])


        self.node_dict["base_crash"] = self.generate_OrCPT(["base_crash_by_close_to_obj"],["not_crash", "crash"])    #ロボットが衝突する確率


        self.node_dict["map_is_broken_by_base_crash"] = ConditionalProbabilityTable(#ロボットの衝突が原因で地図が壊れる確率
            [["crash", "True", 0.9],
            ["crash", "False", 0.1],
            ["not_crash", "True", 0],
            ["not_crash", "False", 1]],
            [self.node_dict["base_crash"]])
 


    
        self.node_dict["map_is_broken_by_slam_problem"] = ConditionalProbabilityTable(#ロボットの衝突が原因で地図が壊れる確率
            [["True", "True", 1],
            ["True", "False", 0],
            ["False", "True", 0],
            ["False", "False", 1]],
            [self.node_dict["fewer_features"]])
    

        self.node_dict["map_is_broken"] = self.generate_OrCPT(["map_is_broken_by_base_crash", "map_is_broken_by_slam_problem"],["not_broken", "broken"])#地図が壊れる確率

        


        self.node_dict["base_pose_trip"] = ConditionalProbabilityTable(#自己位置が急に大きく変わる確率
            [["broken", "True", 0.8],
            ["broken", "False", 0.2],
            ["not_broken", "True", 0.1],
            ["not_broken", "False", 0.9]],
            [self.node_dict["map_is_broken"]])
        self.observable_node_list.append("base_pose_trip")


        self.node_dict["path_not_found_by_broken_map"] = ConditionalProbabilityTable(#地図の壊れ具合によるけどとりあえず
            [["broken", "True", 0.8],
            ["broken", "False", 0.2],
            ["not_broken", "True", 0],
            ["not_broken", "False", 1]],
            [self.node_dict["map_is_broken"]])


        self.node_dict["ComputePathToPose_without_dynamic_obj"] = self.generate_OrCPT(["path_not_found_by_broken_map", "path_not_found_by_no_path"],["found", "not_found"])   




        self.node_dict["path_not_found_by_dynamic_object"] = ConditionalProbabilityTable(
            [["found", "True", 1/2],
            ["found", "False", 1/2],
            ["not_found", "True", 0.1],
            ["not_found", "False", 0.9]],#静的マップで経路が見つからないなら動的障害物が原因ではない（ただし、障害物を除去した地図の生成に問題があると必ずしもその限りではない）
            [self.node_dict["ComputePathToPose_without_dynamic_obj"]])
      




        self.node_dict["ComputePathToPose"] = self.generate_OrCPT(["path_not_found_by_dynamic_object",
                                                                "path_not_found_by_broken_map", 
                                                                "path_not_found_by_no_path"],
                                                                ["found", "not_found"])    
 
        self.action_mapping["ComputePathToPose"]={NodeStatus.SUCCESS:"found", NodeStatus.FAILURE:"not_found"}
        self.observable_node_list.append("ComputePathToPose")

        # path_not_found =ConditionalProbabilityTable(#静的マップで経路が見つかるなら動的障害物が原因で経路が見つからないということ
        #     [["True", "not_found", 1],
        #     ["True", "found", 0],
        #     ["False", "not_found", 1/2],
        #     ["False", "found", 1/2]],
        #     [path_not_found_by_dynamic_object])

        # path_exist = ConditionalProbabilityTable(#静的マップで経路が見つかるなら動的障害物が原因で経路が見つからないということ
        #     [["True", "exist", 1],
        #     ["True", "not_exist", 0],
        #     ["False", "exist", 1/2],
        #     ["False", "not_exist", 1/2]],
        #     [path_not_found_by_dynamic_object])


        """経路追従関連"""
        self.node_dict["FollowPathFailure_by_base_crash"] = ConditionalProbabilityTable(
            [["crash", "True", 0.8],
            ["crash", "False", 0.2],
            ["not_crash", "True", 0],
            ["not_crash", "False", 1]],# 衝突していないなら衝突は経路追従に失敗した原因ではない
            [self.node_dict["base_crash"]])
     
        
        self.node_dict["FollowPathFailure_by_base_control_error"] = ConditionalProbabilityTable(# 制御異常で経路追従できない確率
            [["error", "True", 0.5],
            ["error", "False", 0.5],
            ["no_error", "True", 0],
            ["no_error", "False", 1]],# 制御に異常ないなら制御異常が原因ではない
            [self.node_dict["base_control_error"]])

        self.node_dict["FollowPath"] = self.generate_OrCPT(["FollowPathFailure_by_base_crash", 
                                        "FollowPathFailure_by_base_control_error"],["SUCCESS", "FAILURE"])  #ベースの経路追従に成功する確率
        self.action_mapping["FollowPath"]={NodeStatus.SUCCESS:"SUCCESS", NodeStatus.FAILURE:"FAILURE"}


  

        # stuck_by_close_to_obj = ConditionalProbabilityTable(#立ち往生する原因が障害物に近すぎることが原因の確率
        #     [["close", "True", 1/2],
        #     ["close", "False", 1/2],
        #     ["not_close", "True", 0],
        #     ["not_close", "False", 1]],
        #     [close_to_obj])
        # self.node_dict["stuck_by_close_to_obj"]=stuck_by_close_to_obj

        # stuck_by_broken_costmap = ConditionalProbabilityTable(#立ち往生する原因がコストマップの異常の場合
        #     [["broken", "True", 1/2],
        #     ["broken", "False", 1/2],
        #     ["not_broken", "True", 0],
        #     ["not_broken", "False", 1]],
        #     [costmap_is_broken])
        # self.node_dict["stuck_by_broken_costmap"]=stuck_by_broken_costmap






        # robot_is_stuck = self.generate_OrCPT([stuck_by_close_to_obj,stuck_by_broken_costmap],["not_stuck", "stuck"])    
        # self.node_dict["robot_is_stuck"]=robot_is_stuck


        #
    # resultは結果ノードの状態名を入力する。[正常,異常]  の順
    def generate_OrCPT(self,cause_name_list, rusult):
        table =[]
        key_list = []
        cause_dist_list = []
        
        for name in cause_name_list:
            self.cause_list.append(name)
            cause_dist_list.append(self.node_dict[name])

        for dist in cause_dist_list:
            keys = dist.keys()
            key_list.append(list(keys))
            # print()
        key_list.append(rusult)
        rows = list(itertools.product(*[keys for keys in key_list]))
        for row in rows:
            row = list(row)
            if len(list(set(row[:-1]))) == 1 and list(set(row[:-1]))[0] == "False":#原因がすべてFalseの場合
                if row[-1] == rusult[1]:
                    row.append(0)
                else:
                    row.append(1)

            elif row[-1] == rusult[1]:
                row.append(1)
            else:
                row.append(0)
            table.append(row)
        return ConditionalProbabilityTable(table,cause_dist_list)


    def set_edge(self):
        for name, node in self.node_dict.items():
            if isinstance(node, ConditionalProbabilityTable):
                parents = []
                for parent_node in node.parameters[1]:
                    parents += get_keys_from_value(self.node_dict, parent_node)
                for parent_name in parents:
                    # print((parent_name, name))   
                    self.edge_list.append((parent_name, name))

    def get_parents(self, node_name):
        parents = []
        for parent_node in self.node_dict[node_name].parameters[1]:
            parents += get_keys_from_value(self.node_dict, parent_node)
        return parents


def get_keys_from_value(d, val):
    return [k for k, v in d.items() if v == val]


if __name__ == "__main__":
    # pa_set = NodeSet().node_dict["grasp_slip"].to_dict()["parents"]
    test = NodeSet()

    
    
    # for parent_node in test.node_dict["img_undetected_error"].parameters[1]:
    #     parents += get_keys_from_value(test.node_dict, parent_node)
    # print(parents)


    #   ノードの親を記録
    # for name, node in test.node_dict.items():
    #     if isinstance(node, ConditionalProbabilityTable):
    #         parents = []
    #         for parent_node in node.parameters[1]:
    #             parents += get_keys_from_value(test.node_dict, parent_node)
    #         print(name, parents)
                # for name2, node2 in test.node_dict.items():
                #     if parent_node == node2:
                #         print(name, name2)

            # print(type(node.parameters[1]))
            # print(node)
            # if node.parameters[1]==test.node_dict["grasp_slip"]:
            #     print(name)
            # for pa in node.distribution.parameters[1]:
            #     # if pa == NodeSet().node_dict["grasp_slip"]:
            #     print(type(pa))




    # def get_keys_from_value(d, val):
    #     return [k for k, v in d.items() if v == val]
    # print()
    # cpt = generate_OrCPT([path_not_found_by_dynamic_object,path_not_found_by_broken_map],["found", "not_found"])
    # print(path_not_found_by_no_path)

    
    # print(base_crash_by_close_to_obj.to_dict()["parents"])  #CPTの親を表示
