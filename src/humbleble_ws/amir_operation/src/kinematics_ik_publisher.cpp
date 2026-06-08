#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "kinematics.h"

class IKPublisher : public rclcpp::Node
{
public:
  IKPublisher()
  : Node("ik_publisher")
  {
    publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("ik_joint_states", 10);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&IKPublisher::timer_callback, this)
    );

    // ロボットアームのリンク長
    kinematic_ = std::make_shared<DOF5Kinematic>(0.31, 0.31, 0.31, 0.1);

    // ターゲットのエンドエフェクタ位置・姿勢
    target_pose_ = DOF5Kinematic::Pos5D_t(0.3, 0.0, 0.2, 0.0, 0.0, 0.0); // [m], [rad]
  }

private:
  void timer_callback()
  {
    DOF5Kinematic::IKSolves_t ik_solutions;
    bool success = kinematic_->solveIK(target_pose_, ik_solutions);

    if (!success) {
      RCLCPP_WARN(this->get_logger(), "Inverse Kinematics Failed.");
      return;
    }

    // ここでは最初の解だけ使う
    const auto& joints = ik_solutions.config[0];

    sensor_msgs::msg::JointState msg;
    msg.header.stamp = this->now();
    msg.name = {"joint1", "joint2", "joint3", "joint4", "joint5"};

    // solveIKの出力は[rad]なのでそのまま使える
    msg.position.resize(5);
    for (int i = 0; i < 5; i++) {
      msg.position[i] = joints.a[i] * 1000; // 単位はrad
    }

    publisher_->publish(msg);

    // 4つの解をすべてパブリッシュ
    // for (int solution_idx = 0; solution_idx < 4; ++solution_idx)
    // {
    //   const auto& joints = ik_solutions.config[solution_idx];

    //   sensor_msgs::msg::JointState msg;
    //   msg.header.stamp = this->now();
    //   msg.name = {"joint1", "joint2", "joint3", "joint4", "joint5"};

    //   msg.position.resize(5);
    //   for (int i = 0; i < 5; i++) {
    //     msg.position[i] = joints.a[i] * 1000; // ミリラジアン単位で出力（※注意）
    //   }

    //   publisher_->publish(msg);

    //   // ちょっとだけsleep入れるとわかりやすい（なくてもいい）
    //   rclcpp::sleep_for(std::chrono::milliseconds(100));
    // }
  }

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<DOF5Kinematic> kinematic_;
  DOF5Kinematic::Pos5D_t target_pose_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IKPublisher>());
  rclcpp::shutdown();
  return 0;
}
