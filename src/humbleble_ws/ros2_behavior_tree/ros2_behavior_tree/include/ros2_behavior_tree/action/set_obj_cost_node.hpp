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

#ifndef ROS2_BEHAVIOR_TREE__ACTION__SET_COST_OBJ_NODE_HPP_
#define ROS2_BEHAVIOR_TREE__ACTION__SET_COST_OBJ_NODE_HPP_

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
#include "my_nav_msgs/srv/set_obj_cost.hpp"

namespace ros2_behavior_tree
{

class SetObjCostNode : public BT::AsyncActionNode
{
public:
  SetObjCostNode(const std::string & name, const BT::NodeConfiguration & config)
  : BT::AsyncActionNode(name, config)
  {
    ros2_node_ = BT::TreeNode::config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    request_ = std::make_shared<my_nav_msgs::srv::SetObjCost::Request>();
    response_ = std::make_shared<my_nav_msgs::srv::SetObjCost::Response>();
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
      BT::InputPort<std::vector<moveit_msgs::msg::CollisionObject>>("objects", "collisions"),
      BT::InputPort<std::string>("command", "ADD, REMOVE, RESET"),
    };
  }

  BT::NodeStatus tick() override
  {
    if (!getInput("service_name", service_name_)) {
      throw BT::RuntimeError("Missing parameter [service_name] in ROS2ServiceClientNode");
    }
    
    if (!getInput("command", command_)) {
      throw BT::RuntimeError("Missing parameter [command] in SetObjCostNode");
    }

    std::vector<moveit_msgs::msg::CollisionObject> objects;
    if (!getInput<std::vector<moveit_msgs::msg::CollisionObject>>("objects", objects)) {
      throw BT::RuntimeError("Missing parameter [objects] in SetObjCostNode");
    }

    if (service_client_ == nullptr) {
      service_client_ = ros2_node_->create_client<my_nav_msgs::srv::SetObjCost>(service_name_);
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
    
    set_marker(objects);
    


    //結果を受信するまで待機
    // Send the request to the server
    auto future_result_ = service_client_->async_send_request(request_);

    rclcpp::FutureReturnCode rc;

    do
    {
      rc = callback_group_executor_.spin_until_future_complete(future_result_, server_timeout_);
      if (rc == rclcpp::FutureReturnCode::SUCCESS) {
        response_ = future_result_.get();
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


  void set_marker(const std::vector<moveit_msgs::msg::CollisionObject> & objects)
  {
    request_->markers.markers.clear();
    for(auto obj: objects)
    {
      visualization_msgs::msg::Marker marker;
      if(command_=="ADD")marker.action = visualization_msgs::msg::Marker::ADD;
      else if(command_=="REMOVE")marker.action = visualization_msgs::msg::Marker::DELETE;
      else if(command_=="CLEAR")marker.action = visualization_msgs::msg::Marker::DELETEALL;
      else throw BT::RuntimeError("invalid command in SetObjCostNode");

      marker.header.frame_id = "map";
      marker.pose = obj.pose;
      marker.type = visualization_msgs::msg::Marker::CYLINDER;
      marker.scale.x = 0.1;
      marker.text = obj.id;
      // marker.scale.y = 0.1;
      // marker.scale.z = 0.1;

      request_->markers.markers.push_back(marker);
    }
    request_->partial_update=true;
    // builtin_interfaces::msg::Time time;
    // time.sec = 0;
    // time.nanosec = 0;
    // marker.header.stamp = time;
    // marker.header.frame_id = "map";

    // // Set the namespace and id for this marker. This serves to create a unique ID
    // // Any marker sent with the same namespace and id will overwrite the old one
    // marker.ns = "follower_goal";
    // marker.id = 1;

    // marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    // marker.action = visualization_msgs::msg::Marker::ADD;

    // // Set the scale of the marker -- 1x1x1 here means 1m on a side
    // marker.scale.x = 0.1;
    // marker.scale.y = 0.1;
    // marker.scale.z = 0.1;

    // // 0 indicates the object should last forever
    // marker.lifetime = rclcpp::Duration::from_nanoseconds(0);

    // marker.frame_locked = false;

    // marker.points.resize(poses.size());
    // marker.colors.resize(poses.size());

    // std_msgs::msg::ColorRGBA color;
    // color.r = 1.0;
    // color.a = 0.5;

    // for (const auto & p : poses) {
    //   geometry_msgs::msg::Pose pose;
    //   pose.orientation.w = 1.0;
    //   pose.position.x = p.pose.position.x;
    //   pose.position.y = p.pose.position.y;
    //   marker.points.push_back(pose.position);
    //   marker.colors.push_back(color);
    //   color.r = color.r >= 0.25 ? color.r - 0.25 : 1.0;
    // }

    // marker_pub_->publish(markers);
  }

protected:
  bool initialized_{false};
  std::shared_ptr<rclcpp::Node> ros2_node_;
  // rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  std::chrono::milliseconds server_timeout_;
  rclcpp::Client<my_nav_msgs::srv::SetObjCost>::SharedPtr service_client_;
  std::shared_ptr<my_nav_msgs::srv::SetObjCost::Request> request_;
  std::shared_ptr<my_nav_msgs::srv::SetObjCost::Response> response_;
  std::shared_future<std::shared_ptr<my_nav_msgs::srv::SetObjCost_Response>> future_result_;
  std::string service_name_;
  std::string command_;
};

}  // namespace ros2_behavior_tree

#endif  // ROS2_BEHAVIOR_TREE__ACTION__SET_COST_OBJ_NODE_HPP_
