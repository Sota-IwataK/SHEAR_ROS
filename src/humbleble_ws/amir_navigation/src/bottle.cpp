#include <rclcpp/rclcpp.hpp>
#include <darknet_ros_msgs/msg/bounding_boxes.hpp>
#include "amir_interfaces/msg/amir_sensor.hpp"

class BBoxFlagNode : public rclcpp::Node
{
public:
  BBoxFlagNode()
  : Node("bbox_flag_node"), bottle_detected_(false), latest_angle0_(0.0)
  {
    // QoS 設定: BEST_EFFORT（データ損失を許容）
    rclcpp::QoS qos_profile(10); // キューサイズ10
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

    // bounding_boxesの購読
    bbox_sub_ = this->create_subscription<darknet_ros_msgs::msg::BoundingBoxes>(
      "/darknet_ros/bounding_boxes", 10,
      std::bind(&BBoxFlagNode::bbox_callback, this, std::placeholders::_1));

    // AmirSensorの購読
    angle_sub_ = this->create_subscription<amir_interfaces::msg::AmirSensor>(
      "/encoder_pub", qos_profile,
      std::bind(&BBoxFlagNode::angle_callback, this, std::placeholders::_1));
  }

private:
  void bbox_callback(const darknet_ros_msgs::msg::BoundingBoxes::SharedPtr msg)
  {
    // ボトルを検出した場合のみ処理を行う
    for (const auto& box : msg->bounding_boxes) {
      if (box.class_id == "bottle" && !bottle_detected_) {
        bottle_detected_ = true;
        // ボトル検出時に角度を表示
        RCLCPP_INFO(this->get_logger(), "🍾 bottle を検出！検出角度: angle[0] = %.1f", latest_angle0_);
      }
    }
  }

  void angle_callback(const amir_interfaces::msg::AmirSensor::SharedPtr msg)
  {
    // 最新の角度を記録
    latest_angle0_ = msg->angle[0];
  }

  bool bottle_detected_;  // ボトル検出フラグ
  float latest_angle0_;   // 最新の角度

  rclcpp::Subscription<darknet_ros_msgs::msg::BoundingBoxes>::SharedPtr bbox_sub_;
  rclcpp::Subscription<amir_interfaces::msg::AmirSensor>::SharedPtr angle_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BBoxFlagNode>());
  rclcpp::shutdown();
  return 0;
}
