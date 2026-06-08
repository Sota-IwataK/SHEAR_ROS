#ifndef ROS2_BEHAVIOR_TREE__ACTION__SET_MULTI_PATH_HPP_
#define ROS2_BEHAVIOR_TREE__ACTION__SET_MULTI_PATH_HPP_

#include <string>
#include <memory>

#include "behaviortree_cpp_v3/action_node.h"
#include "ros2_behavior_tree/bt_conversions.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logger.hpp"
#include "moveit_msgs/msg/collision_object.hpp"
#include "my_nav_msgs/msg/path_with_condition.hpp"

namespace ros2_behavior_tree
{

class SetMultiPath : public BT::SyncActionNode
{
public:
  SetMultiPath(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::vector<my_nav_msgs::msg::PathWithCondition>>("input_multi_path", ""),
      BT::InputPort<nav_msgs::msg::Path>("path", ""),
      BT::InputPort<std::vector<moveit_msgs::msg::CollisionObject>>("collisions", "collisions"),
      BT::OutputPort<std::vector<my_nav_msgs::msg::PathWithCondition>>("output_multi_path", "my_nav_msgs::msg::PathWithCondition")
    };
  }

  BT::NodeStatus tick() override
  {
    nav_msgs::msg::Path path;
    if (!getInput<nav_msgs::msg::Path>("path", path)) {
      throw BT::RuntimeError("Missing parameter [path] in SetMultiPath");
    }

    std::vector<moveit_msgs::msg::CollisionObject> collisions;
    if (!getInput<std::vector<moveit_msgs::msg::CollisionObject>>("collisions", collisions)) {
      throw BT::RuntimeError("Missing parameter [collisions] in SetMultiPath");
    }

    //  入力にmulti_pathのリストがあればそれで初期化
    std::vector<my_nav_msgs::msg::PathWithCondition> multi_path;
    getInput<std::vector<my_nav_msgs::msg::PathWithCondition>>("input_multi_path", multi_path);


    //  経路と条件を追加
    my_nav_msgs::msg::PathWithCondition path_with_condititon;
    path_with_condititon.path = path;
    for(auto collision: collisions)
      path_with_condititon.remove_id_set.push_back(collision.id);
    multi_path.push_back(path_with_condititon);

    
    
    setOutput<std::vector<my_nav_msgs::msg::PathWithCondition>>("output_multi_path", multi_path);
    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace ros2_behavior_tree

#endif  // ROS2_BEHAVIOR_TREE__ACTION__SET_MULTI_PATH_HPP_
