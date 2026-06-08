#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <string>
#include <math.h>

class JointPublisher : public rclcpp::Node
{
public:
  JointPublisher()
  : Node("vis_joint_publisher"), count_(0)
  {
    joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100), std::bind(&JointPublisher::publish_joint_states, this));
  }

private:
  void publish_joint_states()
  {
    auto js0 = sensor_msgs::msg::JointState();
    js0.header.stamp = this->get_clock()->now();
    js0.name.resize(15);
    js0.name[0] = "Joint_1";
    js0.name[1] = "Joint_2";
    js0.name[2] = "Joint_3";
    js0.name[3] = "Joint_4";
    js0.name[4] = "Joint_5";
    js0.name[5] = "Gripper";
    js0.name[6] = "left_inner_back_joint";
    js0.name[7] = "right_outer_back_joint";
    js0.name[8] = "left_outer_back_joint";
    js0.name[9] = "right_inner_front_joint";
    js0.name[10] = "left_inner_front_joint";

    //mecanum
    js0.name[11] = "front_left_joint";
    js0.name[12] = "front_right_joint";
    js0.name[13] = "back_right_joint";
    js0.name[14] = "back_left_joint";

    js0.position.resize(15);
    js0.position[0] = 0;
    js0.position[1] = 1.0;
    js0.position[2] = -2.0;
    js0.position[3] = 1.0;
    // js0.position[3] = -(js0.position[1]+js0.position[2]);
    js0.position[4] = 0;
    js0.position[5] = 0;
    js0.position[6] = -0.0000004;
    js0.position[7] = -0.0000004;
    js0.position[8] = -0.0000004;
    js0.position[9] = -0.0000004;
    js0.position[10] = -0.0000004;
    js0.position[11] = 0;
    js0.position[12] = 0;
    js0.position[13] = 0;
    js0.position[14] = 0;

    
    joint_pub_->publish(js0);
    count_++;
  }

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  int count_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointPublisher>());
  rclcpp::shutdown();
  return 0;
}
