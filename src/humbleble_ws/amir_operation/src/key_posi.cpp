
#include <rclcpp/rclcpp.hpp>
#include <amir_interfaces/msg/amir_cmd.hpp>
#include <termios.h>
#include <unistd.h>
#include <iostream>
#include <string>
#include <vector>

class JointPublisher : public rclcpp::Node
{
public:
  JointPublisher()
  : Node("vis_joint_publisher")
  {
    rclcpp::QoS qos_profile(rclcpp::KeepLast(10));
    qos_profile.best_effort();
    joint_pub_ = this->create_publisher<amir_interfaces::msg::AmirCmd>("/motor_sub", qos_profile);
    angle_ = {-2967.0, -40.0, 400.0, -1700.0, 2748.0, -1300.0};
    vel_   = {400.0, 200.0, 400.0, 400.0, 400.0, 200.0};
  }

  void run()
  {
    rclcpp::Rate rate(10);
    std::cout << "=== 操作説明 ===\n";
    std::cout << "[1~6] ジョイント選択\n";
    std::cout << "[↑] 増加, [↓] 減少, [ESC] 終了\n";

    int joint_index = 0;
    while (rclcpp::ok()) {
      char c = get_key();
      if (c == 27) break;  // ESCで終了

      if (c >= '1' && c <= '6') {
        joint_index = c - '1';
        std::cout << "ジョイント " << joint_index + 1 << " 選択\n";
      } else if (c == 'A') {  // 上矢印キー
        angle_[joint_index] += 100;
      } else if (c == 'B') {  // 下矢印キー
        angle_[joint_index] -= 100;
      }

      publish_joint_states();
      rate.sleep();
    }
  }

private:
  char get_key()
  {
    struct termios oldt, newt;
    char c;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    c = getchar();
    if (c == 27 && getchar() == '[') c = getchar();  // 矢印キー対応
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return c;
  }

  void publish_joint_states()
  {
    auto joint = amir_interfaces::msg::AmirCmd();
    for (int i = 0; i < 6; ++i) {
      joint.angle[i] = angle_[i];
      joint.vel[i] = vel_[i];
    }

    for (int i = 0; i < 5; ++i) {  // 5回連続送信
      joint_pub_->publish(joint);
      rclcpp::sleep_for(std::chrono::milliseconds(20));
    }
    RCLCPP_INFO(this->get_logger(), "🌀ジョイント角度送信中...");
  }

  rclcpp::Publisher<amir_interfaces::msg::AmirCmd>::SharedPtr joint_pub_;
  std::vector<float> angle_;
  std::vector<float> vel_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JointPublisher>();
  node->run();  // spin_someじゃなくてループに変更
  rclcpp::shutdown();
  return 0;
}
