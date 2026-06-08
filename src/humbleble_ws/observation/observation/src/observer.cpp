#include <rclcpp/rclcpp.hpp>
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include <sensor_msgs/msg/laser_scan.hpp>
// #include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <observation_msgs/msg/path_evaluation.hpp>
#include <observation_msgs/srv/path_check.hpp>

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

// class CloseToObstacle : public rclcpp::Node
// {
// public:
//   CloseToObstacle()
//   : Node("close_to_obstacle")
//   {
//     scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
//       "scan", rclcpp::SensorDataQoS(),
//       std::bind(&CloseToObstacle::handle_scan, this, _1));
//   }

// private:
//   std::string robot_namespace_;

//   void handle_scan(const std::shared_ptr<sensor_msgs::msg::LaserScan> msg)
//   {
//     sensor_msgs::msg::LaserScan pub_msg;
 
//   }

//   rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
// };

/*---------------------------------------------------------------------------*/


// class SubCostMap : public rclcpp::Node
// {
// public:
//   SubCostMap()
//   : Node("SubCostMap")
//   {
//     cost_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
//       "global_costmap/costmap", rclcpp::SensorDataQoS(),
//       std::bind(&SubCostMap::cost_cb, this, _1));
//   }

// private:
//   std::string robot_namespace_;

//   void cost_cb(const std::shared_ptr<nav_msgs::msg::OccupancyGrid> msg)
//   {
//     costmap_ = *msg;
//     msg->header.frame_id;
//     // msg->data
//     // costmap_ = msg->getCostmap();
//     // costmap_->getCost()
//     // nav_msgs::msg::OccupancyGrid pub_msg;
//   }

//   rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr cost_sub_;
//   nav_msgs::msg::OccupancyGrid costmap_;
//   std::string global_frame_;
// };

/*---------------------------------------------------------------------------*/


class NavObservation : public rclcpp::Node
{
public:
  NavObservation()
  : Node("nav_observation")
  {
    path_global_plan_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      "plan", rclcpp::SensorDataQoS(),
      std::bind(&NavObservation::global_plan_cb, this, _1));

    path_local_plan_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      "local_plan", rclcpp::SensorDataQoS(),
      std::bind(&NavObservation::local_plan_cb, this, _1));

    global_cost_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "global_costmap/costmap", rclcpp::SensorDataQoS(),
      std::bind(&NavObservation::global_cost_cb, this, _1));

    local_cost_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "local_costmap/costmap", rclcpp::SensorDataQoS(),
      std::bind(&NavObservation::local_cost_cb, this, _1));

    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "map", rclcpp::SensorDataQoS(),
      std::bind(&NavObservation::map_cb, this, _1));

    path_eva_pub_ = this->create_publisher<observation_msgs::msg::PathEvaluation>("path_evaluation", 1);

    


  }

private:
  std::string robot_namespace_;

  

  void global_cost_cb(const std::shared_ptr<nav_msgs::msg::OccupancyGrid> msg)
  {
    RCLCPP_INFO(get_logger(), "global_cost_cb");
    global_costmap_ = *msg;
  }

  void local_cost_cb(const std::shared_ptr<nav_msgs::msg::OccupancyGrid> msg)
  {
    local_costmap_ = *msg;
  }

  void map_cb(const std::shared_ptr<nav_msgs::msg::OccupancyGrid> msg)
  {
    map_ = *msg;
  }


  void global_plan_cb(const std::shared_ptr<nav_msgs::msg::Path> msg)
  {
    global_plan_ = *msg;

    // for(auto pose : global_plan_.poses)
    // {
    //   get_cost_from_position(global_costmap_, pose.pose.position);
    // }
  }

  void local_plan_cb(const std::shared_ptr<nav_msgs::msg::Path> msg)
  {
    local_plan_ = *msg;

    int max_cost = 0;
    int ave_cost = 0;

    for(auto pose : local_plan_.poses)
    {
      int value = get_cost_from_position(local_costmap_, pose.pose.position);
      ave_cost +=value;
      if(max_cost < value) max_cost = value;
    }
    
    observation_msgs::msg::PathEvaluation path_eva;
    path_eva.max_cost = max_cost;
    path_eva.ave_cost = ave_cost/local_plan_.poses.size();

    path_eva_pub_->publish(path_eva);
  }

  
  /*座標から地図のコストを取得*/
  int get_cost_from_position(nav_msgs::msg::OccupancyGrid map, geometry_msgs::msg::Point position)
  {
    float x_grid = position.x / map.info.resolution;
    float y_grid = position.y / map.info.resolution;

    float origin_x_grid = map.info.origin.position.x / map.info.resolution;
    float origin_y_grid = map.info.origin.position.y / map.info.resolution;

    int dx = x_grid - origin_x_grid;
    int dy = y_grid - origin_y_grid;
    if(dx < 0 || dy < 0)
    {
      RCLCPP_ERROR(get_logger(), "out of cost");
      return -1;
    }

    int index = dy * map.info.width + dx;
    // RCLCPP_INFO(get_logger(), "cost=%d",  map.data[index]);
    return map.data[index];
  }

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_global_plan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_local_plan_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr global_cost_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr local_cost_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Publisher<observation_msgs::msg::PathEvaluation>::SharedPtr path_eva_pub_;
  
  std::string global_frame_;
  std::vector<geometry_msgs::msg::PoseStamped> poses_;
  nav_msgs::msg::OccupancyGrid global_costmap_;
  nav_msgs::msg::OccupancyGrid local_costmap_;
  nav_msgs::msg::OccupancyGrid map_;
  nav_msgs::msg::Path global_plan_;
  nav_msgs::msg::Path local_plan_;
};

/*---------------------------------------------------------------------------*/


class RobotPoseSub : public NavObservation
{
public:
  RobotPoseSub(rclcpp::Node::SharedPtr node):ros2_node_(node)
  // : Node("robot_pose_sub")
  {
    // ros2_node_ = rclcpp::Node::make_shared("testtest");
    tf_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("tf_odom", 1);

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", rclcpp::SensorDataQoS(),
      std::bind(&RobotPoseSub::odom_cb, this, _1));


    tf_buffer_ =
      std::make_unique<tf2_ros::Buffer>(this->get_clock());
    transform_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    timer_ = this->create_wall_timer(
      20ms, std::bind(&RobotPoseSub::on_timer, this));

    path_check_server_ = ros2_node_->create_service<observation_msgs::srv::PathCheck>(
      "path_check",
      std::bind(&RobotPoseSub::path_check_cb, this, _1, _2));
  }

private:
  std::string robot_namespace_;

  void path_check_cb(
    const std::shared_ptr<observation_msgs::srv::PathCheck::Request> request,
    const std::shared_ptr<observation_msgs::srv::PathCheck::Response> response)
  {
    rclcpp::sleep_for(10s);

  }

  void odom_cb(const std::shared_ptr<nav_msgs::msg::Odometry> msg)
  {
    odom_ = *msg;
  }

  void on_timer()
  {
    // std::string fromFrameRel = "map";
    // std::string toFrameRel = "base_link";

    std::string fromFrameRel = "base_link";
    std::string toFrameRel = "map";
    try {
          transformStamped_ = tf_buffer_->lookupTransform(
            toFrameRel, fromFrameRel,
            tf2::TimePointZero);
    } catch (tf2::TransformException & ex) {
      RCLCPP_INFO(
        this->get_logger(), "Could not transform %s to %s: %s",
        toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
      return;
    }




    float delta_tf = sqrt(pow(transformStamped_.transform.translation.x -b_tf_.transform.translation.x,2)
      + pow(transformStamped_.transform.translation.y -b_tf_.transform.translation.y,2));
      
    
    // float delta_odom = sqrt(pow(odom_.pose.pose.position.x -b_odom_.pose.pose.position.x,2)
    //   + pow(odom_.pose.pose.position.y -b_odom_.pose.pose.position.y,2));

    // RCLCPP_INFO(get_logger(), "delta_odom:%lf, delta_tf:%lf",delta_odom,delta_tf);

    //  自己位置表示
    // double map_x = transformStamped_.transform.translation.x;
    // double map_y = transformStamped_.transform.translation.y;
    // RCLCPP_INFO(get_logger(), "map_x:%lf, map_y:%lf",map_x,map_y);
    
    // double odom_x = odom_.pose.pose.position.x;
    // double odom_y = odom_.pose.pose.position.y;
    // RCLCPP_INFO(get_logger(), "odom_x:%lf, odom_y:%lf",odom_x,odom_y);

    // double gap = sqrt(pow(map_x-odom_x,2) + pow(map_y-odom_y,2));
    // RCLCPP_INFO(get_logger(), "gap:%lf",gap);

    uint32_t delta_time = (transformStamped_.header.stamp.nanosec - b_tf_.header.stamp.nanosec);
    
  
    


    b_odom_ = odom_;
    if(delta_time != 0)
    {
      // RCLCPP_INFO(get_logger(), "delta_time:%lf",float(delta_time)/ 1000000000);
      // RCLCPP_INFO(get_logger(), "vel:%lf",delta_tf / (float(delta_time)/ 1000000000));
      tf_odom_.header.stamp = transformStamped_.header.stamp;
      tf_odom_.header.frame_id = fromFrameRel;
      tf_odom_.pose.pose.position.x = transformStamped_.transform.translation.x;
      tf_odom_.pose.pose.position.y = transformStamped_.transform.translation.y;
      tf_odom_.pose.pose.position.z = transformStamped_.transform.translation.z;
      tf_odom_.pose.pose.orientation = transformStamped_.transform.rotation;
      tf_odom_.twist.twist.linear.x = (transformStamped_.transform.translation.x - b_tf_.transform.translation.x) / (float(delta_time)/ 1000000000);
      tf_odom_.twist.twist.linear.y = (transformStamped_.transform.translation.y - b_tf_.transform.translation.y) / (float(delta_time)/ 1000000000);
      
       // tf_odom_.twist.twist.linear.x = transformStamped_.transform.translation.x -b_tf_.translation.x;
      tf_odom_pub_->publish(tf_odom_);
      
      b_tf_ = transformStamped_;
    }
  }

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr tf_odom_pub_;
  rclcpp::TimerBase::SharedPtr timer_{nullptr};
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::string target_frame_;
  geometry_msgs::msg::TransformStamped transformStamped_;
  nav_msgs::msg::Odometry odom_;
  nav_msgs::msg::Odometry b_odom_;  // １つ前のオドメトリ
  geometry_msgs::msg::TransformStamped b_tf_;  // １つ前のtf自己位置
  nav_msgs::msg::Odometry tf_odom_;
  rclcpp::Service<observation_msgs::srv::PathCheck>::SharedPtr path_check_server_;
  rclcpp::Node::SharedPtr ros2_node_;
};


// class Observer : public NavObservation
// {
//   public:
//     Observer()
//     {
//       rclcpp::executors::SingleThreadedExecutor exec;
//       auto node1 = std::make_shared<NavObservation>();
//       exec.add_node(node1);
//       auto node2 = std::make_shared<RobotPoseSub>();
//       exec.add_node(node2);
//       exec.spin();
//     }

// }

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor exec;
  auto node1 = rclcpp::Node::make_shared("testtest");
  // auto node1 = std::make_shared<NavObservation>();
  exec.add_node(node1);
  auto node2 = std::make_shared<RobotPoseSub>(node1);
  // node2->

  
  exec.add_node(node2);

  exec.spin();



  // auto node = std::make_shared<PathSub>();
  // rclcpp::spin(node->get_node_base_interface());
  // rclcpp::spin(std::make_shared<RobotPoseSub>());
  // rclcpp::spin(std::make_shared<NavObservation>());

  // rclcpp::spin(std::make_shared<CloseToObstacle>());
  rclcpp::shutdown();
  return 0;
}