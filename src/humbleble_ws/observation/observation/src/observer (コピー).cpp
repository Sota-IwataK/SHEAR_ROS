#include <rclcpp/rclcpp.hpp>
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include <sensor_msgs/msg/laser_scan.hpp>
// #include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include <nav_msgs/msg/path.hpp>
using std::placeholders::_1;

class CloseToObstacle : public rclcpp::Node
{
public:
  CloseToObstacle()
  : Node("close_to_obstacle")
  {
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", rclcpp::SensorDataQoS(),
      std::bind(&CloseToObstacle::handle_scan, this, _1));
  }

private:
  std::string robot_namespace_;

  void handle_scan(const std::shared_ptr<sensor_msgs::msg::LaserScan> msg)
  {
    sensor_msgs::msg::LaserScan pub_msg;
 
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
};

/*---------------------------------------------------------------------------*/


// class SubCostMap : public rclcpp::Node
// {
// public:
//   SubCostMap()
//   : Node("close_to_obstacle")
//   {
//     cost_sub_ = this->create_subscription<nav2_costmap_2d::Costmap2DROS>(
//       "scan", rclcpp::SensorDataQoS(),
//       std::bind(&SubCostMap::cost_cb, this, _1));
//   }

// private:
//   std::string robot_namespace_;

//   void cost_cb(const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> msg)
//   {
//     costmap_ = msg->getCostmap();
//     costmap_->getCost()
//     // nav2_costmap_2d::Costmap2DROS pub_msg;
 
//   }

//   rclcpp::Subscription<nav2_costmap_2d::Costmap2DROS>::SharedPtr cost_sub_;
//   nav2_costmap_2d::Costmap2D * costmap_;
//   std::string global_frame_;
// };

/*---------------------------------------------------------------------------*/


class PathSub : public rclcpp_lifecycle::LifecycleNode
{
public:
  PathSub()
  : rclcpp_lifecycle::LifecycleNode("path_sub")
  {
    
  }

private:
  std::string robot_namespace_;

  void path_cb(const std::shared_ptr<nav_msgs::msg::Path> msg)
  {
    RCLCPP_INFO(get_logger(), "path_cb");
    geometry_msgs::msg::Point b_position;
    for(auto pose : msg->poses)
    {
      // pose.pose.position
      b_position = pose.pose.position;
    }

    // nav_msgs::msg::Path pub_msg;
 
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override
  {
    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      "local_plan", rclcpp::SensorDataQoS(),
      std::bind(&PathSub::path_cb, this, _1));
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override {};
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override {};
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override {};
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override {};
  CallbackReturn on_error(const rclcpp_lifecycle::State & state) override {};

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  // nav2_costmap_2d::Costmap2D * costmap_;
  std::string global_frame_;
  std::vector<geometry_msgs::msg::PoseStamped> poses_;
};

/*---------------------------------------------------------------------------*/
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor exe;

  std::shared_ptr<PathSub> ps_node =
    std::make_shared<PathSub>();

  exe.add_node(ps_node->get_node_base_interface());
  exe.spin();



  // auto node = std::make_shared<PathSub>();
  // rclcpp::spin(node->get_node_base_interface());


  // rclcpp::spin(std::make_shared<CloseToObstacle>());
  rclcpp::shutdown();
  return 0;
}