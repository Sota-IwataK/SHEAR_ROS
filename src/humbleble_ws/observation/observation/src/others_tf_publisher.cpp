#include "rclcpp/rclcpp.hpp"
#include <string>
#include <chrono>
#include <memory>
#include <functional>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <forward_list>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
using std::placeholders::_1;
using namespace std::chrono_literals;



struct PoseSubscription {
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr map_sub;
  std::string robot_name;
};



class OthersTfPub : public rclcpp::Node
{
    public:
        OthersTfPub()
        : Node("others_tf_publisher")
        {
            // Call on_timer function every second
            pose_subscribing_timer_ = this->create_wall_timer(
                20ms, std::bind(&OthersTfPub::on_timer, this));
        }

    private:
        void on_timer()
        {     
            
        }
        // std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        rclcpp::TimerBase::SharedPtr pose_subscribing_timer_{nullptr};
        std::forward_list<PoseSubscription> subscriptions_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<OthersTfPub>();
    // rclcpp::WallRate rate(1s);
    // while (rclcpp::ok()) 
    // {
        
    //     rclcpp::spin_some(node);
    //     rate.sleep();
    // }
    // rclcpp::spin(std::make_shared<GetGlobalParam>());
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

// int main(int argc, char **argv)
// {
//     rclcpp::init(argc, argv);
//     rclcpp::NodeOptions options;
//     options.allow_undeclared_parameters(true);
//     options.automatically_declare_parameters_from_overrides(true);

//     auto node = std::make_shared<rclcpp::Node>("global_parameter_server", options);
//     
//     node->get_parameter("robot_list", robot_list);

//     rclcpp::spin(node);
//     rclcpp::shutdown();
//     return 0;
// }