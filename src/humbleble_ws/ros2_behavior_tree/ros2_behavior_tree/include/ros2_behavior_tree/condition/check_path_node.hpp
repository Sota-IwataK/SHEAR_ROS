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

#ifndef ROS2_BEHAVIOR_TREE__CONDITION__CHECK_PATH_NODE_HPP_
#define ROS2_BEHAVIOR_TREE__CONDITION__CHECK_PATH_NODE_HPP_

#include <string>
#include <memory>

#include "behaviortree_cpp_v3/condition_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "observation_msgs/msg/object.hpp"


namespace ros2_behavior_tree
{

class CheckPathNode : public BT::ConditionNode
{
public:
  CheckPathNode(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ConditionNode(name, config)
  {
  }

  CheckPathNode() = delete;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<nav_msgs::msg::Path>("path", ""),
      BT::InputPort<std::vector<moveit_msgs::msg::CollisionObject>>("collisions", ""),

      BT::OutputPort<std::vector<moveit_msgs::msg::CollisionObject>>("collisions_on_path", ""),
    };
  }

  BT::NodeStatus tick() override
  {
    nav_msgs::msg::Path path;
    if (!getInput<nav_msgs::msg::Path>("path", path)) {
      throw BT::RuntimeError("Missing parameter [path] in CheckPath node");
    }

    std::vector<moveit_msgs::msg::CollisionObject> collisions;
    if (!getInput<std::vector<moveit_msgs::msg::CollisionObject>>("collisions", collisions)) {
      throw BT::RuntimeError("Missing parameter [collisions] in SetObjCostNode");
    }


    std::vector<moveit_msgs::msg::CollisionObject> collisions_on_path;
    for(auto pose: path.poses)
    {
      for(auto collision: collisions)
      {
        double distance 
        = sqrt(pow(pose.pose.position.x - collision.pose.position.x,2) 
            + pow(pose.pose.position.y - collision.pose.position.y, 2));

        if(distance < 0.5)
          collisions_on_path.push_back(collision);
        
      }
    }

    // double outer_radius;
    // if (!getInput<double>("outer_radius", outer_radius)) {
    //   throw BT::RuntimeError("Missing parameter [outer_radius] in CheckPath node");
    // }

    // double inner_radius;
    // if (!getInput<double>("inner_radius", inner_radius)) {
    //   inner_radius = 0;
    // }

    

    setOutput<std::vector<moveit_msgs::msg::CollisionObject>>("collisions_on_path", collisions_on_path);


    if(collisions_on_path.size() == 0)
    {
      return BT::NodeStatus::SUCCESS;
    }
    else
    {
      return BT::NodeStatus::FAILURE;
    }

  }
};

}  // namespace ros2_behavior_tree

#endif  // ROS2_BEHAVIOR_TREE__CONDITION__CHECK_PATH_NODE_HPP_
