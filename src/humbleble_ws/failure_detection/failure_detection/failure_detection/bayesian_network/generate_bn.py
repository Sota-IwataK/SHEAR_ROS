# import os
# print('basename:    ', os.path.basename(__file__))
# print('dirname:     ', os.path.dirname(__file__))
# import sys

# sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
from pomegranate import *
import numpy as np
import math
# import networkx as nx
import matplotlib.pyplot as plt
import matplotlib.image as matimg
import tempfile
from failure_detection_msgs.msg import Solution
import pygraphviz
from ..bayesian_network.MyNode import NodeSet
# from MyNode import NodeSet #単体検証用

def plot(model):
	G = pygraphviz.AGraph(directed=True)

	for state in model.states:
		G.add_node(state.name, color='red')

	for parent, child in model.edges:
		G.add_edge(parent.name, child.name)

	with tempfile.NamedTemporaryFile() as tf:
		G.draw(tf, format='png', prog='dot')
		img = matimg.imread(tf)
		plt.imshow(img)
		plt.axis('off')
		plt.show()

def graph_plot(p_dict):
  fig = plt.figure(figsize=(12, 9))

  x_labels = np.array([])
  model = np.array([])
  for key, value in p_dict.items():
    x_labels = np.append(x_labels, (key))
    # x_labels = np.append(x_labels, (key + "{:.2f}".format(value)))
    model = np.append(model, value)
    # print("key:" + mykey + ", values:" + myvalue)


  
  # # 尤度(ベルヌーイ分布)を計算
  # true_model = np.array([0.1, 0.2, 0.7])
  # 尤度を作図
  plt.bar(x=x_labels, height=model, color='purple') # 尤度
  # plt.xlabel(xlabel)
  plt.ylabel('prob')
  # plt.xticks(ticks=x_labels, labels=x_labels) # x軸目盛
  # plt.suptitle(title, fontsize=20)
  # plt.title('$\mu=' + str(mu) + '$', loc='left')
  plt.ylim(False, True)
  plt.show()





def get_keys_from_value(d, val):
    return [k for k, v in d.items() if v == val]





class BN:
    def __init__(self, name):
        self.bn_name = name
        self.initialize()
        
        
        # self.model = BayesianNetwork("test")
        # self.index = {}
        # self.edge_list = []
        # self.node_set = {}# キー：状態変数名、値：Node  ライブラリ内部でNodeをキーにした辞書が存在するため，Nodeは複製してBNに追加してはならない
        # self.node_graph_set = {}
        self.graph_fig = plt.figure(figsize=(20,20))    #   図の大きさ
        plt.rcParams["font.size"] = 15  #   図の文字サイズ
        plt.subplots_adjust(wspace=0.6, hspace=2.0) #   図の隙間調整
        plt.tight_layout()  #要らない

    def initialize(self):
        self.AllNodeSet = NodeSet()
        self.model = None
        self.index = {} #   キー：状態変数名、値：resultの配列のindex
        self.edge_list = []
        self.node_set = {}# キー：状態変数名、値：Node  ライブラリ内部でNodeをキーにした辞書が存在するため，Nodeは複製してBNに追加してはならない
        self.node_graph_set = {}
        self.generate_bn(self.bn_name)


    # def grow_bn(self, model, child_node):
    #     parents = []
    #     for i, node in enumerate(child_node.distribution.parameters[1]): #  child_nodeの親ノード(確率表)で繰り返し
    #         parents += get_keys_from_value(AllNodeSet.node_dict, node)  #   親ノードの名前を取得して追加

    #         #ノードを作成して保管する
    #         if parents[i] not in self.node_set:
    #             self.node_set[parents[i]] = Node(node, name=parents[i])

    #         parent_node = self.node_set[parents[i]]

    #         #まだノードがなければ追加する
    #         if parents[i] not in self.index:
    #             self.index[parents[i]] = model.state_count()#index保存
    #             model.add_states(parent_node)
    #         #まだエッジがなければ追加する
    #         if (parents[i], child_node.name) not in self.edge_list:
    #             self.edge_list.append((parents[i], child_node.name))
    #             model.add_edge(parent_node, child_node)
    #         #まだ親がいれば成長させる
    #         if isinstance(node, ConditionalProbabilityTable):
    #             self.grow_bn(model, parent_node)

    def grow_bn(self, model, node_name):
        for edge in self.AllNodeSet.edge_list:
            if node_name in edge:
                if edge[0] not in self.node_set:
                    self.node_set[edge[0]] = Node(self.AllNodeSet.node_dict.get(edge[0], None), name=edge[0])
                    self.index[edge[0]] = model.state_count()#index保存
                    model.add_states(self.node_set[edge[0]])
                   
                    

                if edge[1] not in self.node_set:
                    self.node_set[edge[1]] = Node(self.AllNodeSet.node_dict.get(edge[1], None), name=edge[1])
                    self.index[edge[1]] = model.state_count()#index保存
                    model.add_states(self.node_set[edge[1]])

                #まだエッジがなければ追加する
                if edge not in self.edge_list:
                    # print(edge)
                    self.edge_list.append(edge)
                    model.add_edge(self.node_set[edge[0]], self.node_set[edge[1]])
                else:
                    continue

                
                if node_name == edge[0]:
                        self.grow_bn(model, edge[1])
                elif node_name == edge[1]:
                        self.grow_bn(model, edge[0])

                
                # else:
                #     self.grow_bn(model, edge[0])







    #入力：結果のノード名
    #子から親をに向かってBNを生成する
    def generate_bn(self, result):
        self.model = BayesianNetwork(result)
        self.bn_name = result
        self.grow_bn(self.model, result)
        self.model.bake()#計算準備

    def predict_proba(self, dict={}):
        return self.model.predict_proba(dict)
    


    def plot(self):
        plt.figure()
        plot(self.model)
        plt.show()

    # 事後確率のグラフの表示
    def graph_plot(self, result):
        grid_size = math.ceil(math.sqrt(len(self.index)))
        # print("grid_size",grid_size)
        self.graph_fig.clf()
        for node_name, index in self.index.items():
            # if node_name not in self.node_graph_set:
            self.node_graph_set[node_name] = self.graph_fig.add_subplot(grid_size, grid_size, index+1)
            # else:
            #     self.node_graph_set[node_name].cla()
            
            self.node_graph_set[node_name].set_xlabel(node_name)
            self.node_graph_set[node_name].set_ylabel('prob')
            self.node_graph_set[node_name].set_ylim([0,1])
                # 
        for i, dist in enumerate(result):
            node_name2 = get_keys_from_value(self.index, i)
            x_labels = np.array([])
            model = np.array([])
            if isinstance(dist, DiscreteDistribution):
                for key, value in dist.to_dict()["parameters"][0].items():
                    x_labels = np.append(x_labels, (key))
                    # x_labels = np.append(x_labels, (key + "{:.2f}".format(value)))
                    model = np.append(model, value)
                rect = self.node_graph_set[node_name2[0]].bar(x=x_labels, height=model, color='purple') # 尤度
                # rect = self.node_graph_set[node_name2[0]].barh(y=x_labels, width=model, color='purple', align="center") # 横棒
                self.node_graph_set[node_name2[0]].bar_label(rect, labels=[f'{x:.1%}' for x in rect.datavalues], label_type='edge') #   数値表示
                # print(node_name2[0], dist.to_dict()["parameters"])
            else:
                # print(node_name2[0], dist)#観測
                self.node_graph_set[node_name2[0]].text(0.4, 0.4, dist, size=20, color="red")    #観測した状態名の表示
        # plt.pause(0.01)

    def graph_show(self):
        plt.pause(0.01)
        



    def print_probability(self, result, node_name):
        print(node_name, result[self.index[node_name]].to_dict()["parameters"])
        # graph_plot(result[self.index[node_name]].to_dict()["parameters"][0])

    def print_all_probability(self, result):
        for i, dist in enumerate(result):
            node_name = get_keys_from_value(self.index, i)
            if isinstance(dist, DiscreteDistribution):
                print(node_name[0], dist.to_dict()["parameters"])
                # graph_plot(dist.to_dict()["parameters"][0])
            else:
                print(node_name[0], dist)#観測
            
        self.graph_plot(result) # グラフを表示する
        # self.update_probability(result)


    #   事前確率を事後確率で更新   
    def update_probability(self, result):
        for i, dist in enumerate(result):
            node_name = get_keys_from_value(self.index, i)
            if isinstance(dist, DiscreteDistribution):
                
                self.AllNodeSet.node_dict[node_name[0]] = dist
        # AllNodeSet.update()
        # self.initialize()
        # self.generate_bn(self.bn_name)

    def get_parents(self, node_name):
        return self.AllNodeSet.get_parents(node_name)

    #   結果の原因を高確率順で取得
    def get_cause_dict(self, result_name, result):
        cause_dict = {}
        pa_list = self.get_parents(result_name)
        pa_list = np.array(pa_list)

        true_probability_list = np.full(len(pa_list), -1.0) #    原因ノード以外は確率を取得しない（-1固定）
        for i, name in enumerate(pa_list):
            if name in self.AllNodeSet.cause_list:  #   原因ノード以外は無視
                index = self.index[name]
                true_probability_list[i] = result[index].to_dict()["parameters"][0]["True"] #

        sorted_index_list = np.argsort(true_probability_list)[::-1] #   降順

        # print(len(sorted_index_list))

        for index in sorted_index_list:
            cause_dict[pa_list[index]] = true_probability_list[index]
            
        return cause_dict

    def search_cause(self, result_name, result, all_cause_dict):
        if isinstance(self.AllNodeSet.node_dict[result_name], ConditionalProbabilityTable):   
            cause_dict = self.get_cause_dict(result_name, result)
            all_cause_dict.update(cause_dict)
            for name in cause_dict.keys():
                self.search_cause(name, result, all_cause_dict)



            

    def get_all_cause_dict(self, result_name, result):
        all_cause_dict = {}
        self.search_cause(result_name, result, all_cause_dict)
        # print(all_cause_dict)
        all_cause_dict = {k: v for k, v in all_cause_dict.items() if v != -1}   #   原因ノードだけにする

        
        # all_cause_dict = {k: v for k, v in all_cause_dict.items() if k in self.AllNodeSet.solution_dict.keys()}   #   解決策が存在するBNノードのみ（）
        # print(all_cause_dict)
        # for cause_list in all_cause_list:
        #     print("----")
        #     print(cause_list)
        
        return all_cause_dict
            

    def get_solutions(self, all_cause_dict, threhold = 0.1):
        all_cause_dict = {k: v for k, v in all_cause_dict.items() if k in self.AllNodeSet.solution_dict.keys()}   #   解決策が存在するBNノードのみ（）
        all_cause_list = sorted(all_cause_dict.items(), key=lambda x:x[1], reverse=True)    #   確率の高い順にソート
        solutions = []
        for cause in all_cause_list:
            if cause[1] > threhold:
                cause_name = cause[0]
                recovery = self.AllNodeSet.solution_dict[cause_name]
                solutions.append(recovery)
        return solutions

    




        


     
    
if __name__ == "__main__":
    # print(node.node_list)
    model = BN("undetected_error")
    # model.generate_bn("grasp_result")
    # model.generate_bn("place_result")
    # model.generate_bn("ComputePathToPose")
    # model.generate_bn("ComputePathToPose_without_dynamic_obj")
    # print(model.index)
    # print(model.edge_list)
    
    # result = model.model.predict_proba({"ComputePathToPose":"not_found", "ComputePathToPose_without_dynamic_obj":"found"})
    # result = model.model.predict_proba({"ComputePathToPose_without_dynamic_obj": "not_found","base_pose_trip":"False"})
    # print(result)
    # model.print_probability(result, "ComputePathToPose")

    # for i in range(50):
    #     result = model.model.predict_proba({"ComputePathToPose":"not_found", "base_pose_trip":"True"})
    #     model.print_all_probability(result)
    #     # model.plot()

    # result = model.model.predict_proba({"ComputePathToPose":"not_found", "base_pose_trip":"True"})
    # result = model.model.predict_proba({"ComputePathToPose":"not_found", "base_pose_trip":"True", "close_to_obj":"local_cost<90"})
    result = model.model.predict_proba({"undetected_error": "error",
                                        "arm_2_joint_0": "abs<20deg",
                                        "arm_collision": "no_collision"})
    # print(result)
    
    all_cause_list = model.get_all_cause_dict("undetected_error", result)
    print(all_cause_list)
    solutions = model.get_solutions(all_cause_list)
    print(solutions)

    

    
    # model.get_cause_list("img_undetected_error", result)
    # pa_list = model.get_parents("img_undetected_error")

 

    model.print_all_probability(result)
    model.plot()

    # model2 = BN()
    # model2.generate_bn("ComputePathToPose")
    # result = model2.model.predict_proba({"ComputePathToPose":"not_found", "base_pose_trip":"True"})
    # model2.print_all_probability(result)
    # # print(result[model.index["ComputePathToPose"]].to_dict()["parameters"])
    # # result = model.model.predict_proba({'place_result': "lying"})
    # # result = model.model.predict_proba({"target_weight": "normal"})
    # # print(result[model.index["place_result"]])
    
    # model2.plot()

    # plt.figure()
    # plot(bn)
    # plt.show()
    # model = generate_bn("place_result")
    # model.bake()
    # for state in :
    #     print(type(state["distribution"]))
    # aaa = model.predict_proba({})
    # print(aaa)
    # plt.figure()
    # plot(model)
    # plt.show()
    