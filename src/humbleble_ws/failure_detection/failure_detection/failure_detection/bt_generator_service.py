import sys
from typing import no_type_check_decorator
from lxml.builder import E
from lxml import etree
import rclpy
from rclpy.node import Node

import numpy as np
from ros2_behavior_tree_msgs.srv import GetBT
# import actionlib
# from behaviortree_ros.msg import BTExecutorAction, BTExecutorGoal
# from behaviortree_ros.srv import GetBT, GetBTResponse
# from behaviortree_ros.srv import ErrorDetection
# from geometry_msgs.msg import PoseStamped, Point
# from youbot_projected_gradient.msg import object_pose
# from youbot_projected_gradient.srv import get_objects, get_obj


# path_check_xml = 'IsPathValid'
# print_xml = '<PrintValue message="some_message"/>'
# # some_xml_data = '<BehaviorTree ID="MoveRobot"><Fallback><SequenceStar><MoveBase       goal="{target}"/><SetBlackboard output_key="output" value="mission accomplished" /></SequenceStar></Fallback></BehaviorTree>'
# # element = etree.fromstring(some_xml_data)

# move_base_xml = '<MoveBase server_name="mecanum/move_base" x="0" y ="0" frame="mecanum/map"/>'

# # pick_xml = '<PickProjected server_name="bt_pick" id="else" pose_x="0.0" pose_y="-1.0" pose_z="0.1" /> '
# # place_xml = '<PlaceProjected server_name="bt_pick" id="else" pose_x="0.0" pose_y="-1.0" pose_z="0.1" />'
# # approach_xml = '<ApproachProjected server_name="bt_approach" id="else" pose_x="0.91" pose_y="-1.0" pose_z="0.1" />'
# # detect_xml = '<DetectIPP server_name="youbot_search" id="else" pose_x="0.91" pose_y="-1.0" pose_z="0.1" />'
# # request_assist_xml = '<AssistIPP server_name="youbot_search" id="else" pose_x="0.91" pose_y="-1.0" pose_z="0.1" />'

# pick_xml = '<PickProjected server_name="bt_pick" node_id="0" id="else" pose = "0.0;0.0;0.0" input="{approachedResult}" output="{picked}"/>'
# place_xml = '<PlaceProjected server_name="/bt_place" node_id="0" id="none" location="none" at="0.0; 0.0; 0.0" />'
# approach_xml = '<ApproachProjected server_name="bt_approach" node_id="0" id="else" pose="0.0; 0.0; 0.0" type="none" output="{approachedResult}" result="{approachedCode}"/>'
# search_ipp_xml = '<ExploreIPP service_name="/youbot/self_explore" trajectory="{traj}"  />'

# detect_xml = '<DetectIPP server_name="youbot_search" id="else" pose_x="0.91" pose_y="-1.0" pose_z="0.1" />'
# request_assist_xml = '<VisualAssist server_name="visual_assist" type="path" path="{traj}" at="{location}" />'


# is_path_valid_xml ='<IsPathValid service_name="gridmap/trajEntropy" node_id="0" pose="0.0;0.0;0.0" segment="{traj}" />'
# is_path_coll_free_xml ='<IsPathCollFree service_name="coll_check_srv" node_id="0" id="else" pose="0.0;0.0;0.0"/>'
# is_grasped_xml ='<IsGraspedObj service_name="gripper_state_server" node_id="0" id="None" />'
# is_hand_xml ='<IsHandFree service_name="gripper_state_server" node_id="0" id="None" />'
# is_robot_close_xml ='<IsRobotCloseTo service_name="is_robot_close_to" node_id="0" id="None" pose="0.0;0.0;0.0" />'

# is_obj_at_xml ='<IsObjectAt service_name="are_objects_close" node_id="0" id="None" location="None" />'
# # is_obj_found_xml ='<IsObjectAt service_name="are_objects_close" object="None" pose="None" />'



class ActionTemp:
    __slots__ = ['precond', 'action', 'effects', 'xml', 'cos']

    def __init__(self,xml, p, e, a, cos):
        self.precond = p
        self.action =  a
        self.xml = xml
        self.effects =  e
        self.cos = cos

    def to_string(self, property):
        s =""
        for p in property :
            
            s += str(p) 
        return s


class Obstacle:
    __slots__ = ['name', 'position']
    
    def __init__(self, name, position):
        self.name = name
        self.position = position

class Location:
    __slots__ = ['name', 'position', 'detected']
    
    def __init__(self, name, position, detected):
        self.name = name
        self.position = position
        self.detected = detected

class Target:
    __slots__ = ['name', 'position', 'detected']
    
    def __init__(self, name, position, detected):
        self.name = name
        self.position = position
        self.detected = detected

class Belief:
    __slots__ = ['distribution']
    
    def __init__(self, distribution):
        self.distribution = distribution

    def update(self, belief):
        return belief


class Constraint:
    __slots__ = ['type', 'xml', 'params']
    
    def __init__(self, type, xml, params):
        self.type = type
        self.xml = xml        
        self.params = params
    def __copy__(self):
        cls = self.__class__
        newobject = cls.__new__(cls)
        newobject.__dict__.update(self.__dict__)
        newobject.params = self.params
        newobject.xml = self.xml
        newobject.type = self.type
        return newobject

class ConstraintOperativeSubspace:
    def __init__(self, variables, constraints):
        self.variables = variables
        self.constraints = constraints

# class ActionTemplate:
#     def __init__(self,name,xml,parameters,conditions,effects,cos):
#         self.name = name
#         self.xml = xml
#         self.parameters = {}  # dictionary
#         for p in parameters:
#             self.parameters.update({p : None})
#         self.conditions = conditions #list
#         self.effects = effects #list
#         self.cos = cos
#         self.nodeType = 'Action'
#         self.nodeClass = 'Leaf'    


        
# is_obj_detected = Constraint('is_object_at', is_obj_at_xml,{'object': 0, 'detected': 0}) #for detect or so called get observation action 
# is_object_at = Constraint('is_object_at', is_obj_at_xml,{'object': 0, 'at': 0}) 
# is_robot_close_to = Constraint('is_robot_close_to',is_robot_close_xml, {'robot': 0, 'to': 0})
# is_grasped = Constraint('is_grasped',is_grasped_xml, {'object': 0, 'hand':0})
# is_hand_free = Constraint('is_hand_free',is_hand_xml, {'empty': 0, 'hand':0})
# is_path_valid = Constraint('is_path_valid',is_path_valid_xml, {'path': 0, 'object': 0})
# is_path_coll_free = Constraint('is_path_coll_free',is_path_coll_free_xml, {'path': 0, 'object': 0})

# constraints= []
# constraints.append(is_obj_detected)
# constraints.append(is_object_at)
# constraints.append(is_robot_close_to)
# constraints.append(is_hand_free)
# constraints.append(is_grasped)

# constraints.append(is_path_valid)


# #ActionTemp(xml, precond, effects, action, cos)
# # approach_to = ActionTemp(approach_xml,[is_path_coll_free,is_path_valid],['robot','to'],'ApproachProjected', 'pose2d')
# # approach_to = ActionTemp(approach_xml,[is_path_valid],['robot','to'],'ApproachProjected', 'pose2d')
# approach_to = ActionTemp(approach_xml,[is_path_coll_free],['robot','to'],'ApproachProjected', 'pose2d')
# detect_obj = ActionTemp(detect_xml,[],['object','found'],'DetectIPP','o')
# assist_detect_obj = ActionTemp(request_assist_xml,[],['path'],'VisualAssist','e' )
# pick_obj = ActionTemp(pick_xml,[is_hand_free,is_robot_close_to],['object','hand'],'PickProjected', 'o')
# place_obj = ActionTemp(place_xml,[is_grasped,is_robot_close_to],['object','at'],'PlaceProjected', 'at')

# # drop_obj = ActionTemp(place_xml,[],['hand','empty'],'drop_obj', {'at': 0})
# explore_path = ActionTemp(search_ipp_xml,[],['path'],'ExploreIPP','e')

# actions = []
# actions.append(approach_to)
# # actions.append(detect_obj)
# # actions.append(assist_detect_obj)
# actions.append(pick_obj)
# actions.append(place_obj)

# # actions.append(drop_obj)
# actions.append(explore_path)

# ### -----------------------------------------------------------------------------------------------------

# # def get_pose_by_id(object_id):
        
# #     check_pose = object_pose()
# #     check_pose.coordinate.x = 1.0
# #     check_pose.coordinate.x = 2.0
# #     check_pose.coordinate.x = 3.0

# #     return check_pose

# storage_pose = {"pose": "0.0;-2.0;0.0", "type": "storage"}
# obj_pose = {"pose": "1.0;-1.0;0.0", "type": "target_obj"}

# storage_pose2 = {"pose": "0.0;-2.0;0.0", "type": "storage"}
# obj_pose2 = {"pose": "1.0;-1.0;0.0", "type": "target_obj"}

# storage_pose3 = {"pose": "0.0;-2.0;0.0", "type": "storage"}
# obj_pose3 = {"pose": "1.0;-1.0;0.0", "type": "target_obj"}

# globalObjs = {
#   "storage": storage_pose,
#   "target": obj_pose,
#   "storage2": storage_pose2,
#   "target2": obj_pose2,
#   "storage3": storage_pose3,
#   "target3": obj_pose3
# }



# globalObjs = {
#   "storage": "0.5;-1.0;0.0",
#   "target": "0.5;-2.5;0.0",
# }




# dict = {}
# parallel = []
# def getXmlFromConditionString(goal_condition):
#     goalCondition = etree.fromstring(goal_condition)
#     for c in constraints :
#         element = etree.fromstring(c.xml)
#         if goalCondition.tag == element.tag :
#             condition = c
    
#     condition.xml = etree.tostring(goalCondition, encoding="unicode", pretty_print=True)
#     return goalCondition, condition

# def getConditionFromXml(xml):
#     condition = None
#     # print("xml = ", xml)
#     for c in constraints :
#         element = etree.fromstring(c.xml)
#         if xml.tag == element.tag :
#             condition = c
#             condition.xml = etree.tostring(xml, encoding="unicode", pretty_print=True)
#             # print("condition found type = ", condition.type)
#     return condition


# class BTGenerator:
#     def __init__(self):
#         self.nodeID = 0#どのノードか判別する用
    



#     def createInitialTree(self, goal_conditions):
#         root = etree.Element("root")
#         bt = etree.Element("BehaviorTree")
#         sequenceStar = etree.Element("Sequence") #最初をシーケンスにすることで複数の成約を満たせるようにする
        
#         bt.append(sequenceStar)

#         for goal_condition in goal_conditions:
#             fallback = etree.Element("Fallback")
#             # condition = Constraint('is_object_at', is_obj_at_xml,{'object': 0, 'at': 0})
#             # condition.params['object'] = goal_condition[0]
#             # condition.params['at'] = goal_condition[1]
            
#             # goalCondition.set("object1",str(goal_condition[0]))
#             # goalCondition.set("object2",str(goal_condition[1]))
#             goalCondition, condition = getXmlFromConditionString(goal_condition)
#             self.nodeID += 1
#             goalCondition.attrib['node_id']=str(self.nodeID)
            
#             # print("goalCondition=",goalCondition)#goalCondition= <Element IsObjectAt at 0x7f9dc3a02a08>
#             # print("condition=",conditions)#condition= <Element IsObjectAt at 0x7f9dc3a02a08>
#             fallback.append(goalCondition)
#             # print("here")
#             sub_bt = self.get_subtree(condition)#サブツリー生成
#             fallback.append(sub_bt)
#             sequenceStar.append(fallback)



#         root.append(bt)   
#         tree = etree.tostring(root, encoding="unicode", pretty_print=True)
#         print(tree)
#         return tree

#     def predict_error(self, failed_action):	
#         rospy.wait_for_service('/error_detection')
#         try:
#             error_detection_client = rospy.ServiceProxy('/error_detection', ErrorDetection)
            
#             # req1.visualSupport = [w1]
#             # resp1 = bt(req1.type, req1.bt, req1.goal_condition )
#             req = ErrorDetection()
#             req.error_name = failed_action
#             req.related_action_name = ""
#             resp = error_detection_client(req.error_name, req.related_action_name)
#             # print(resp1.behavior_tree)
#             print(resp.result)

#         except rospy.ServiceException as e:
#             print("Service call failed: %s"%e)

#         return resp.result

#     #元のツリー、失敗したノード
#     def updateTree(self, tree, goal_condition, failedNodeID):
#         print(tree)
#         print("!!Failed_condition = ",goal_condition)
#         if (str(goal_condition) == "PickProjected" or str(goal_condition) == "PlaceProjected" or str(goal_condition) == "ApproachProjected"):
#             self.predict_error(goal_condition)

#         root = etree.fromstring(tree)
#         # tree = etree.tostring(root, encoding="unicode", pretty_print=True)
#         # foundElement = root.findall(".//%s" % str(goal_condition))
#         foundElement = []
#         for element in root.iter("%s" %str(goal_condition)):
#             if element.attrib['node_id']==str(failedNodeID):
#                 foundElement.append(element)
#                 print("element=",element.attrib)

#         found = None
#         # print(foundElement)
#         for i in foundElement :
#             if i.tag == "IsRobotCloseTo"  :
#                 print("foundElement.attrib",i.attrib)
#                 dict[i] = [i, i.tag, i.attrib['pose']]
#                 found = i
#                 break
#             elif i.tag == "IsPathValid" :
#                 dict[i] = [i, i.tag]
#                 found = i
#                 print ("found IsPathValid")
#             else :
#                 if 'id' in i.attrib :
#                     dict[i.tag] = i.attrib['id']
#                 else :
#                     dict[i] = [i, i.tag]
#                 found = i
#                 break
#         if found.tag == "IsRobotCloseTo"  :
#             for key in dict :
#                 if dict[key][1] == "IsRobotCloseTo" :
#                     found = dict[key][0]
#         # print("found= ", found)
#         if found is None :
#             print("not found ...")
#             return tree
#         index = found.getparent().index(found)
#         # print('index= ', index)
#         sequenceStar = etree.Element("Sequence") 
#         fallback = etree.Element("Fallback")
#         parent = found.getparent()
#         #満たしていない成約をサブツリーに置き換える
#         found.getparent().remove(found)
#         fallback.append(found)

#         #障害物だったらどける
#         if found.tag == "IsPathCollFree"  :
#             print("Path is not Free!!!")
#         # print("[]: ", found)
#         # print("found=",found)   #found= <Element IsRobotCloseTo at 0x7f0de0533d48>

#         condition = getConditionFromXml(found)
#         if not condition :
#             return tree
#         sub_bt = self.get_subtree(condition)
#         fallback.append(sub_bt)
#         sequenceStar.append(fallback)
#         parent.insert(index, sequenceStar)
#         tree = etree.tostring(root, encoding="unicode", pretty_print=True)
#         return tree
        


#     def createInitialTreeMecanum(self, waypoints):
#         root = etree.Element("root")
#         bt = etree.Element("BehaviorTree")
#         root.append(bt)
#         sequence = etree.Element("Sequence")
#         fallback = etree.Element("Fallback")
#         sequence.append(fallback)
#         i = 0
#         for w in waypoints:
#             move_base_action = etree.fromstring(move_base_xml)
#             move_base_action.set("x",str(w.pose.position.x))
#             move_base_action.set("y",str(w.pose.position.y))
#             print_action = etree.fromstring(print_xml)
#             sequence.append(print_action)
#         bt.append(sequence)
        
#         pathCheck = etree.Element("IsPathValid")
#         fallback.append(pathCheck)
#         tree = etree.tostring(root, encoding="unicode", pretty_print=True)

#         return tree
#     ############################################################################################################
#     def get_subtree(self, constraint):
#             bt = None
#             fallback = etree.Element("Fallback")
#             # print(constraint.type)
#             temp = []
#             for action in actions:
#                 # print('Trying with action', action.name, 'Effects', action.effects, 'Condition fluents', fluent.parameters_dict.keys())
#                 #アクションの効果に成約を満たすものがある場合(同じアクションでも目標が違えば別のものとみなすべきでは？)
#                 if set(action.effects).issubset(set(constraint.params.keys())):
                    
#                     # print('The action ', action.name, ' can hold ', fluent.name)
                    
                    
#                     action_xml_set = self.set_action_values_from_condition(action, constraint)

#                     for action_xml in action_xml_set:
#                         #アクションの事前条件追加
#                         bt = etree.Element("Sequence")
#                         for c in action.precond:
#                             c_xml = self.set_condition_values_from_action(c, action_xml)
#                             bt.append(c_xml)
#                         #最後にアクションを追加
#                         bt.append(action_xml)
#                         fallback.append(bt)
#                     return fallback
#                         # print("action_xml=", action_xml)  #action_xml= <Element PlaceProjected at 0x7f786d52db08>
#                         # temp.append(action_xml)
#             #成約を満たし得るアクションが複数ある場合
#             if len(temp) > 1 and not parallel:
#                 if constraint.type == "is_path_valid" :
#                     tree =  etree.Element("Parallel")
#                     print("setting parallel");
#                     for t in temp :
#                         tree.append(t)
#                     tree.set("success_threshold","1")
#                     tree.set("failure_threshold","1")
#                     parallel.append(tree)
#                     return tree
#                 #通常はfallbackで順に生成（最適化の余地あり）
#                 tree =  etree.Element("Fallback")
#                 for t in temp :
#                     tree.append(t)
#                 return tree
#             if bt is None:
#                 raise Exception('Cannot find action with effects', constraint.params.keys())
#             return bt

   
#     def get_poses_by_group(self, group_id):	
#         rospy.wait_for_service('/get_obj_by_type')
#         try:
#             error_detection_client = rospy.ServiceProxy('/get_obj_by_type', get_objects)
#             resp = error_detection_client(group_id)
#             print(resp.target_poses)

#         except rospy.ServiceException as e:
#             print("Service call failed: %s"%e)

#         return resp.target_poses

#     def get_pose_by_id(self, search_id):	
#         rospy.wait_for_service('/get_obj')
#         try:
#             get_pose_by_id_client = rospy.ServiceProxy('/get_obj', get_obj)
#             resp = get_pose_by_id_client(str(search_id))

#         except rospy.ServiceException as e:
#             print("Service call failed: %s"%e)

#         return resp.target_pose

#     def set_action_values_from_condition(self, action, condition):
#         action_xml_set =[]
        
#         c = etree.fromstring(condition.xml)
        
        
#         #グループの場合該当するidを取得する
#         c_pose_set=[]
#         if 'id' in c.attrib :
#             if c.attrib['id'].startswith('GROUP_'):
#                 group_id = c.attrib['id'].replace('GROUP_', '')
#                 c_pose_set = self.get_poses_by_group(group_id)
#             else:
                
#                 search_id = c.attrib['id']
#                 print("input id=",search_id)
#                 pose = self.get_pose_by_id(search_id)
#                 c_pose_set.append(pose)
#                 print("pose=",pose)

#         for object in c_pose_set:
#             action_xml = etree.fromstring(action.xml)
#             self.nodeID +=1
#             action_xml.attrib['node_id']=str(self.nodeID)
#             if action.cos == "pose2d" :
#                 if 'id' in c.attrib :
                    
#                     action_xml.attrib['id'] = object.id
#                     action_xml.attrib['pose'] = str(object.coordinate.x) + ";"+ str(object.coordinate.y) + ";" + str(object.coordinate.z)
#                     action_xml.attrib['type'] = object.type
#                 # action_xml.attrib['pose'] = c.attrib['pose']
#                 # print(" ### true ....", c.tag)
#                 # print(c.attrib['pose'])
#             elif action.cos == "o" :
#                 if 'id' in c.attrib :
#                     action_xml.attrib['id'] = object.id
#                 # if 'pose' in c.attrib:
#                 #     action_xml.attrib['pose'] = c.attrib['pose']
#                 #     if name.find('obstacle') != -1:
#                 #         action_xml.attrib['pose'] = c.attrib['pose']
#                 # else :
#                 #     action_xml.attrib['id'] = c.attrib['id']

#             elif action.cos == "at" :
#                 action_xml.attrib['location'] = c.attrib['location']
#                 action_xml.attrib['id'] = object.id
#                 action_xml.attrib['at'] = globalObjs[action_xml.attrib['location']]["pose"]
        
#             action_xml_set.append(action_xml)
#         return action_xml_set

#     def set_condition_values_from_action(self, condition, action_xml):
#         c = etree.fromstring(condition.xml)
#         self.nodeID +=1
#         c.attrib['node_id']=str(self.nodeID)
        
        
#         action = None
#         for a in actions :
#             if a.action == action_xml.tag :
#                 action = a
#         print("action found = ", action.action)
#         if action.cos == "at" :

#             if 'id' in c.attrib:
#                 c.attrib['id'] = action_xml.attrib['id']
#             if 'pose' in c.attrib and 'id' in c.attrib:
#                 c.attrib['id'] = action_xml.attrib['location']
#                 c.attrib['pose'] = action_xml.attrib['at']
#                 # if action_xml.attrib['location'] == "storage":
#                 #     # print("true,", " ", action_xml.attrib['id'], " ",  globalObjs[action_xml.attrib['id']])
#                 #     c.attrib['pose'] = globalObjs[action_xml.attrib['location']]
#         else :
#             print("inside else") 
#             if 'id' in c.attrib:
#                 c.attrib['id'] = action_xml.attrib['id']
#                 print("set id")
#             if 'pose' in c.attrib:
#                 c.attrib['pose'] = action_xml.attrib['pose']
#                 # if action_xml.attrib['id'] == "storage" or action_xml.attrib['id'] == "target":
#                 #     # print("true,", " ", action_xml.attrib['id'], " ",  globalObjs[action_xml.attrib['id']])
#                 #     c.attrib['pose'] = globalObjs[action_xml.attrib['id']]["pose"]
#                 #     print("set pose")
        

#         return c

#     ############################################################################################################
#     # def client(tree):
#     #     # Creates the SimpleActionClient, passing the type of the action
#     #     # (FibonacciAction) to the constructor.
#     #     client = actionlib.SimpleActionClient('test_bt', BTExecutorAction)

#     #     # Waits until the action server has started up and started
#     #     # listening for goals.
#     #     client.wait_for_server()

#     #     # Creates a goal to send to the action server.
#     #     goal = BTExecutorGoal()
#     #     goal.behavior_tree = tree
        
#     #     print(goal)
#     #     # Sends the goal to the action server.
#     #     client.send_goal(goal)

#     #     # Waits for the server to finish performing the action.
#     #     client.wait_for_result()

#     #     # # Prints out the result of executing the action
#     #     result = client.get_result()
#     #     print(result)
#     #     # return  result


#     def create_bt(self, req):
#         if req.type == str("new"):
#             tree = GetBTResponse(self.createInitialTree(req.goal_conditions))
#         if req.type == str("update"):
#             tree = GetBTResponse(self.updateTree(req.bt, req.goal_condition, req.failedNodeID))
#         # if len(req.waypoints) > 1 :
#         #     return tree
#         # elif len(req.visualSupport) > 1 :
#         #     return tree 
#         # else :
#         #     tree = GetBTResponse()
#         #     if tree.behavior_tree :
#         #         print("non empty?")
#         #     else :
#         #         print('tree is empty?')
#         return tree

# def create_bts_server():
#     rospy.init_node('bt_generator_service')
#     bt_generator = BTGenerator()
#     s = rospy.Service('get_bt_service', GetBT, bt_generator.create_bt)
#     print("Ready to generate behavior trees")
#     rospy.spin()


class BTGenerator(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        # self.subscription = self.create_subscription(
        #     String,
        #     'topic',
        #     self.listener_callback,
        #     10)
        self.subscription  # prevent unused variable warning

        self.cli = self.create_client(GetBT, 'bt_update')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = GetBT.Request()

        

    # def send_request(self):
    #     self.req.a = int(sys.argv[1])
    #     self.req.b = int(sys.argv[2])
    #     self.future = self.cli.call_async(self.req)


def main(args=None):
    rclpy.init(args=args)
    bt_generator = BTGenerator()
    rclpy.spin(bt_generator)

    rclpy.shutdown()



if __name__ == '__main__':
    main()

