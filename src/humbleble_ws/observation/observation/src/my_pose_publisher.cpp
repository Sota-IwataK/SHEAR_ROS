#include "rclcpp/rclcpp.hpp"
#include <string>
#include <chrono>
#include <memory>
#include <functional>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
using std::placeholders::_1;
using namespace std::chrono_literals;


 
class MyPosePub : public rclcpp::Node
{
    public:
        MyPosePub()
        : Node("my_pose_publisher")
        {
            // pose_publisher_ =
            //     this->create_publisher<geometry_msgs::msg::PoseStamped>("robot_pose", 1);

            tf_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("tf_odom", 1);

            tf_buffer_ =
                std::make_unique<tf2_ros::Buffer>(this->get_clock());
            transform_listener_ =
                std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

            // Call on_timer function every second
            timer_ = this->create_wall_timer(
                20ms, std::bind(&MyPosePub::on_timer, this));
        }

    private:
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
            RCLCPP_WARN(
                this->get_logger(), "Could not transform %s to %s: %s",
                toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
            // return;
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
        // rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_{nullptr};//位置姿勢だけでなくロボットの形状とかも配信すべき？
        std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        geometry_msgs::msg::TransformStamped transformStamped_;
        nav_msgs::msg::Odometry odom_;
        nav_msgs::msg::Odometry b_odom_;  // １つ前のオドメトリ
        geometry_msgs::msg::TransformStamped b_tf_;  // １つ前のtf自己位置
        nav_msgs::msg::Odometry tf_odom_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<MyPosePub>();
    rclcpp::WallRate rate(10);
    while (rclcpp::ok()) 
    {
        
        rclcpp::spin_some(node);
        rate.sleep();
    }

    // rclcpp::spin(node);
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