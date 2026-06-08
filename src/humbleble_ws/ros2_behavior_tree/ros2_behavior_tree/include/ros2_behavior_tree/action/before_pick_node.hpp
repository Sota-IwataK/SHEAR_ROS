#ifndef ROS2_BEHAVIOR_TREE__ACTION__BEFORE_PICK_NODE_HPP_
#define ROS2_BEHAVIOR_TREE__ACTION__BEFORE_PICK_NODE_HPP_

#include <string>
#include <memory>

#include "behaviortree_cpp_v3/action_node.h"
#include "ros2_behavior_tree/bt_conversions.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logger.hpp" 
#include "failure_detection_msgs/srv/before_pick.hpp"
#include "ros2_behavior_tree/bt_service_node.hpp"

namespace ros2_behavior_tree
{

class BeforePickNode : public BtServiceNode<failure_detection_msgs::srv::BeforePick>
{
  public:
    BeforePickNode(const std::string & name, const BT::NodeConfiguration & config)
    : BtServiceNode<failure_detection_msgs::srv::BeforePick>(name, config)
    {
    }

    BeforePickNode() = delete;

    static BT::PortsList providedPorts()
    {
        return providedBasicPorts(
      {
        BT::InputPort<std::string>("obj_id", ""), //  エラー対策
        BT::InputPort<geometry_msgs::msg::PoseStamped>("obj_pose", ""),
      });
    }

    void on_tick()
    {
      if(!getInput<std::string>("obj_id", request_->id))
      {
        throw BT::RuntimeError("Missing parameter [obj_id] in BeforePickNode");
      }

      if (!getInput<geometry_msgs::msg::PoseStamped>("obj_pose", request_->pose)) {
        throw BT::RuntimeError("Missing parameter [obj_pose] in PickNode node");
      }
      increment_recovery_count();
    }


    // BT::NodeStatus on_completion()
    // {
    // }


};

}  // namespace ros2_behavior_tree

#endif  // ROS2_BEHAVIOR_TREE__ACTION__BEFORE_PICK_NODE_HPP_
