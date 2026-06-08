#ifndef ROS2_BEHAVIOR_TREE__ACTION__GET_OBJ_HPP_
#define ROS2_BEHAVIOR_TREE__ACTION__GET_OBJ_HPP_

#include <string>
#include <memory>

#include "behaviortree_cpp_v3/action_node.h"
#include "ros2_behavior_tree/bt_conversions.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logger.hpp" 
#include "observation_msgs/msg/object.hpp"
#include "observation_msgs/srv/get_objects.hpp"

namespace ros2_behavior_tree
{

class GetObjNode : public BT::AsyncActionNode
{
  public:
    GetObjNode(const std::string & name, const BT::NodeConfiguration & config)
    : BT::AsyncActionNode(name, config)
    {
      request_ = std::make_shared<observation_msgs::srv::GetObjects::Request>();
      response_ = std::make_shared<observation_msgs::srv::GetObjects::Response>();
      server_timeout_ = BT::TreeNode::config().blackboard->template get<std::chrono::milliseconds>("server_timeout");
      ros2_node_ =  BT::TreeNode::config().blackboard->template get<std::shared_ptr<rclcpp::Node>>("node");
      callback_group_ = ros2_node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive,
      false);
      callback_group_executor_.add_callback_group(callback_group_, ros2_node_->get_node_base_interface());
    }

    GetObjNode() = delete;

    static BT::PortsList providedPorts()
    {
      return {
        BT::InputPort<std::string>("service_name", "The name of the service to call"),
        BT::InputPort<std::vector<std::string>>("id_set", "The name of the service to call"),
        BT::InputPort<std::string>("command", "ID, GROUP, ALL, ALL_COLLISION"),
        BT::OutputPort<std::vector<moveit_msgs::msg::CollisionObject>>("collisions", "collisions"),
        // BT::InputPort<std::chrono::milliseconds>("server_timeout",
        //   "The timeout value, in milliseconds, to use when waiting for service responses"),
        // BT::InputPort<std::shared_ptr<rclcpp::Node>>("ros2_node",
        //   "The ROS2 node to use when when creating the service")
      };
    }

  BT::NodeStatus tick() override
  {
    if (!getInput("service_name", service_name_)) {
      throw BT::RuntimeError("Missing parameter [service_name] in ROS2ServiceClientNode");
    }

    
    if (!getInput("command", command_)) {
      throw BT::RuntimeError("Missing parameter [command] in ROS2ServiceClientNode");
    }

    if (service_client_ == nullptr) {
      service_client_ = ros2_node_->create_client<observation_msgs::srv::GetObjects>(service_name_);
    }


    // Make sure the server is actually there before continuing
    if (!service_client_->wait_for_service(std::chrono::milliseconds(server_timeout_))) {
      RCLCPP_ERROR(ros2_node_->get_logger(),
        "Timed out waiting for service \"%s\" to become available", service_name_.c_str());
      return BT::NodeStatus::FAILURE;
    }


    if(command_=="ALL_COLLISION")
      request_->command = request_->ALL_COLLISION;
    else if(command_=="ID")
    { 
      request_->command = request_->ID;
    }
    else
    {
      RCLCPP_ERROR(ros2_node_->get_logger(), "invalid input");
    }
      
    //結果を受信するまで待機
    // Send the request to the server
    auto future_result_ = service_client_->async_send_request(request_);

    rclcpp::FutureReturnCode rc;
    rc = callback_group_executor_.spin_until_future_complete(future_result_, server_timeout_);


    if (rc == rclcpp::FutureReturnCode::SUCCESS) {
      response_ = future_result_.get();
      write_output_ports(response_);
      return BT::NodeStatus::SUCCESS;
    } else if (rc == rclcpp::FutureReturnCode::TIMEOUT) {
        RCLCPP_ERROR(ros2_node_->get_logger(), "Failed to set param.");
        return BT::NodeStatus::FAILURE;
    } else {
      RCLCPP_ERROR(ros2_node_->get_logger(),
        "Call to \"%s\" server failed", service_name_.c_str());
      return BT::NodeStatus::FAILURE;
    }

  }

  void write_output_ports(
    std::shared_ptr<observation_msgs::srv::GetObjects::Response> response) 
  {
    std::vector<moveit_msgs::msg::CollisionObject> objects;
    if(command_ == "ALL_COLLISION")
    {
      for(auto obj: response->objects)
      {
        objects.push_back(obj.collision_object);
      }
      
    }
    
    if(!setOutput<std::vector<moveit_msgs::msg::CollisionObject>>("collisions", objects))
          RCLCPP_WARN(ros2_node_->get_logger(),"setOutput failed in %s", service_name_.c_str());
  }

  protected:
    rclcpp::Node::SharedPtr ros2_node_;
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
    std::chrono::milliseconds server_timeout_;
    std::string node_name_;
    rclcpp::Client<observation_msgs::srv::GetObjects>::SharedPtr service_client_;
    std::shared_ptr<observation_msgs::srv::GetObjects::Request> request_;
    std::shared_ptr<observation_msgs::srv::GetObjects::Response> response_;
    std::string service_name_;
    std::string command_;
    std::shared_future<std::shared_ptr<observation_msgs::srv::GetObjects_Response>> future_result_;
};

}  // namespace ros2_behavior_tree

#endif  // ROS2_BEHAVIOR_TREE__ACTION__GET_OBJ_HPP_
