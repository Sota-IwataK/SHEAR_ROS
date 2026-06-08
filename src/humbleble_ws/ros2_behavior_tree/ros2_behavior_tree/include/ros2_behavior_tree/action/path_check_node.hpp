// Copyright (c) 2019 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ROS2_BEHAVIOR_TREE__ACTION__PATH_CHECK_NODE_HPP_
#define ROS2_BEHAVIOR_TREE__ACTION__PATH_CHECK_NODE_HPP_

#include <string>
#include <memory>
#include <vector>
#include <cmath>

#include "behaviortree_cpp_v3/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "builtin_interfaces/msg/duration.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "visualization_msgs/msg/marker_array.hpp"
#include "observation_msgs/msg/object.hpp"
#include "moveit_msgs/msg/collision_object.hpp"
#include "failure_detection_msgs/srv/path_check.hpp"

namespace ros2_behavior_tree
{

class PathCheckNode : public BT::AsyncActionNode
{
public:
  PathCheckNode(const std::string & name, const BT::NodeConfiguration & config)
  : BT::AsyncActionNode(name, config)
  {
    ros2_node_ = BT::TreeNode::config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    request_ = std::make_shared<failure_detection_msgs::srv::PathCheck::Request>();
    response_ = std::make_shared<failure_detection_msgs::srv::PathCheck::Response>();
    server_timeout_ = BT::TreeNode::config().blackboard->template get<std::chrono::milliseconds>("server_timeout");
    callback_group_ = ros2_node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive,
    false);
    callback_group_executor_.add_callback_group(callback_group_, ros2_node_->get_node_base_interface());
    // marker_pub_ = ros2_node_->create_publisher<visualization_msgs::msg::MarkerArray>("object_markers", 1);
  }

  static BT::PortsList providedPorts()
  {
    return {
      // BT::InputPort<geometry_msgs::msg::PoseStamped>("pose",
      //   "The current pose of the robot"),
     
      BT::InputPort<std::string>("service_name", "The name of the service to call"),
      BT::InputPort<nav_msgs::msg::Path>("path", "")

    };
  }

  BT::NodeStatus tick() override
  {
    if (!getInput("service_name", service_name_)) {
      throw BT::RuntimeError("Missing parameter [service_name] in ROS2ServiceClientNode");
    }

    nav_msgs::msg::Path path;
    if (!getInput<nav_msgs::msg::Path>("path", path)) {
      throw BT::RuntimeError("Missing parameter [path] in CheckPath node");
    }
    
   

    if (service_client_ == nullptr) {
      service_client_ = ros2_node_->create_client<failure_detection_msgs::srv::PathCheck>(service_name_);
    }
    // geometry_msgs::msg::PoseStamped pose;
    // if (!getInput<geometry_msgs::msg::PoseStamped>("pose", pose)) {
    //   throw BT::RuntimeError("Missing parameter [pose] in GetPosesNearRobot node");
    // }

    // Make sure the server is actually there before continuing
    if (!service_client_->wait_for_service(std::chrono::milliseconds(server_timeout_))) {
      RCLCPP_ERROR(ros2_node_->get_logger(),
        "Timed out waiting for service \"%s\" to become available", service_name_.c_str());
      return BT::NodeStatus::FAILURE;
    }



    // RCLCPP_INFO(node_->get_logger(),"(x,y) = (%lf, %lf)",pose.pose.position.x, pose.pose.position.y);
    // RCLCPP_INFO(node_->get_logger(),"w = %lf",pose.pose.orientation.w);
    
    
    request_->path = path;

    //結果を受信するまで待機
    // Send the request to the server
    auto future_result_ = service_client_->async_send_request(request_);

    rclcpp::FutureReturnCode rc;

    do
    {
      rc = callback_group_executor_.spin_until_future_complete(future_result_, server_timeout_);
      if (rc == rclcpp::FutureReturnCode::SUCCESS) {
        response_ = future_result_.get();

        if(response_->anomaly_score == -1) return BT::NodeStatus::FAILURE; //異常

        return BT::NodeStatus::SUCCESS;
      } 
      // else if (rc == rclcpp::FutureReturnCode::TIMEOUT) {
      //     RCLCPP_ERROR(ros2_node_->get_logger(), "Failed to set param.");
      //     return BT::NodeStatus::FAILURE;
      // } else {
      //   RCLCPP_ERROR(ros2_node_->get_logger(),
      //     "Call to \"%s\" server failed", service_name_.c_str());
      //   return BT::NodeStatus::FAILURE;
      // }
    } while (rc != rclcpp::FutureReturnCode::SUCCESS && rclcpp::ok());
    
    
    

    
    return BT::NodeStatus::SUCCESS;
  }


 

protected:
  bool initialized_{false};
  std::shared_ptr<rclcpp::Node> ros2_node_;
  // rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  std::chrono::milliseconds server_timeout_;
  rclcpp::Client<failure_detection_msgs::srv::PathCheck>::SharedPtr service_client_;
  std::shared_ptr<failure_detection_msgs::srv::PathCheck::Request> request_;
  std::shared_ptr<failure_detection_msgs::srv::PathCheck::Response> response_;
  std::shared_future<std::shared_ptr<failure_detection_msgs::srv::PathCheck_Response>> future_result_;
  std::string service_name_;
  std::string command_;
};

}  // namespace ros2_behavior_tree

#endif  // ROS2_BEHAVIOR_TREE__ACTION__PATH_CHECK_NODE_HPP_
