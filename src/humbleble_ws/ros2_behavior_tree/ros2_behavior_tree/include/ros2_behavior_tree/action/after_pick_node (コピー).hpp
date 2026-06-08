#ifndef ROS2_BEHAVIOR_TREE__ACTION__AFTER_PICK_NODE_HPP_
#define ROS2_BEHAVIOR_TREE__ACTION__AFTER_PICK_NODE_HPP_

#include <string>
#include <memory>

#include "behaviortree_cpp_v3/action_node.h"
#include "ros2_behavior_tree/bt_conversions.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logger.hpp" 
#include "failure_detection_msgs/srv/after_pick.hpp"


namespace ros2_behavior_tree
{

class AfterPickNode : public BT::AsyncActionNode
{
  public:
    AfterPickNode(const std::string & name, const BT::NodeConfiguration & config)
    : BT::AsyncActionNode(name, config)
    {
      request_ = std::make_shared<failure_detection_msgs::srv::AfterPick::Request>();
      response_ = std::make_shared<failure_detection_msgs::srv::AfterPick::Response>();
      server_timeout_ = BT::TreeNode::config().blackboard->template get<std::chrono::milliseconds>("server_timeout");
      ros2_node_ =  BT::TreeNode::config().blackboard->template get<std::shared_ptr<rclcpp::Node>>("ros2_node");
      callback_group_ = ros2_node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive,
      false);
      callback_group_executor_.add_callback_group(callback_group_, ros2_node_->get_node_base_interface());
    }

    AfterPickNode() = delete;

    static BT::PortsList providedPorts()
    {
      return {
        BT::InputPort<std::string>("service_name", "The name of the service to call"),
        BT::InputPort<std::string>("pick_error_string", ""),
        BT::InputPort<std::string>("subtree_name", ""),
      };
    }

  BT::NodeStatus tick() override
  {
    if (!getInput("service_name", service_name_)) {
      throw BT::RuntimeError("Missing parameter [service_name] in ROS2ServiceClientNode");
    }

    
    // if(!getInput<std::string>("obj_id", goal.id))
    // {
    //   throw BT::RuntimeError("Missing parameter [obj_id] in PickNode node");
    // }
    if (!getInput<std::string>("subtree_name", request_->bt_node_name)) {
      throw BT::RuntimeError("Missing parameter [subtree_name] in AfterPickNode");
    }
    
    if (!getInput<std::string>("pick_error_string", request_->error_string)) {
      throw BT::RuntimeError("Missing parameter [pick_error_string] in AfterPickNode");
    }

    if (service_client_ == nullptr) {
      service_client_ = ros2_node_->create_client<failure_detection_msgs::srv::AfterPick>(service_name_);
    }


    // Make sure the server is actually there before continuing
    if (!service_client_->wait_for_service(std::chrono::milliseconds(server_timeout_))) {
      RCLCPP_ERROR(ros2_node_->get_logger(),
        "Timed out waiting for service \"%s\" to become available", service_name_.c_str());
      return BT::NodeStatus::FAILURE;
    }

      
    //結果を受信するまで待機
    // Send the request to the server
    auto future_result_ = service_client_->async_send_request(request_);

    std::cout << server_timeout_.count() << std::endl;
    rclcpp::FutureReturnCode rc;
    rc = callback_group_executor_.spin_until_future_complete(future_result_, server_timeout_);


    if (rc == rclcpp::FutureReturnCode::SUCCESS) {
      response_ = future_result_.get();
      if(request_->error_string != "success") return BT::NodeStatus::FAILURE; // アクションのエラーコードで結果を決定
      return BT::NodeStatus::SUCCESS;
    } else if (rc == rclcpp::FutureReturnCode::TIMEOUT) {
        RCLCPP_ERROR(ros2_node_->get_logger(), "Failed to set param.");
        // return BT::NodeStatus::FAILURE;
    } else {
      RCLCPP_ERROR(ros2_node_->get_logger(),
        "Call to \"%s\" server failed", service_name_.c_str());
      // return BT::NodeStatus::FAILURE;
    }

    if(request_->error_string=="success") return BT::NodeStatus::SUCCESS; // アクションのエラーコードで結果を決定
    else return BT::NodeStatus::FAILURE;

  }


  protected:
    rclcpp::Node::SharedPtr ros2_node_;
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
    std::chrono::milliseconds server_timeout_;
    std::string node_name_;
    rclcpp::Client<failure_detection_msgs::srv::AfterPick>::SharedPtr service_client_;
    std::shared_ptr<failure_detection_msgs::srv::AfterPick::Request> request_;
    std::shared_ptr<failure_detection_msgs::srv::AfterPick::Response> response_;
    std::string service_name_;
    std::string command_;
    std::string bt_name_;
    std::shared_future<std::shared_ptr<failure_detection_msgs::srv::AfterPick_Response>> future_result_;
};

}  // namespace ros2_behavior_tree

#endif  // ROS2_BEHAVIOR_TREE__ACTION__AFTER_PICK_NODE_HPP_
