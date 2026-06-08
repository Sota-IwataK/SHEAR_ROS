// amir_motion_plan.cpp
//
// MoveIt 2 + OMPL 用のシンプルなゴール送信ノード。
// amir_moveit.launch.py で move_group が動いている状態で実行すると、
// "arm" プランニンググループに目標姿勢を送り、OMPL (RRTConnect) で
// プランニング → そのまま実行する。
//
// 使い方:
//   ros2 run amir_operation amir_motion_plan
//
// 目標姿勢やプランナーを変えたい場合は main() 内の値を編集すれば OK。
//
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <thread>
#include <cmath>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("amir_motion_plan");

  // MoveGroupInterface がサービス呼び出しを行うので executor を回しておく
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  std::thread spinner([&exec]() { exec.spin(); });

  // ----- MoveIt 設定 -----
  static const std::string PLANNING_GROUP = "arm"; // SRDF で定義されたグループ名
  moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);

  // OMPL プランナー指定（ompl_planning.yaml のキー）
  move_group.setPlannerId("RRTConnectkConfigDefault");
  move_group.setPlanningTime(5.0);

  // ----- 目標姿勢 -----
  geometry_msgs::msg::Pose target;
  target.position.x = 0.30;   // [m]
  target.position.y = 0.00;
  target.position.z = 0.40;

  tf2::Quaternion q;
  q.setRPY(0, M_PI / 2, 0);   // roll, pitch, yaw
  target.orientation = tf2::toMsg(q);

  move_group.setPoseTarget(target);

  // ----- プランニング & 実行 -----
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  auto result = move_group.plan(plan);

  if (result == moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_INFO(node->get_logger(), "Plan successful. Executing...");
    move_group.execute(plan);
  }
  else
  {
    RCLCPP_ERROR(node->get_logger(), "Planning failed.");
  }

  rclcpp::shutdown();
  spinner.join();
  return 0;
}
