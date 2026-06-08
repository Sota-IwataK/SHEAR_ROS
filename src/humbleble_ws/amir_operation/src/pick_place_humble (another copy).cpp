/*
 * ROS 2 Humble 向け Pick and Place サンプル
 * amir_mecanum3 (モバイルマニピュレーター) 対応版
 *
 * 座標系の変更点:
 *   旧: world = link0_1 (static_tf: world -> link0_1)
 *   新: world = base_footprint (static_tf: world -> base_footprint)
 *       link0_1 は base_footprint から z = +LINK0_HEIGHT [m] 上に存在する
 *
 *   そのため、旧コードの全ての z 座標に LINK0_HEIGHT を加算する必要がある。
 *   衝突オブジェクトは frame_id = "link0_1" のままにすることで
 *   z 座標の変更を不要にしている。
 */

#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/msg/pose.hpp>
#include <chrono>
#include <vector>
#include <cmath>

using namespace std::chrono_literals;
using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
using PlanningSceneInterface = moveit::planning_interface::PlanningSceneInterface;

// -----------------------------------------------------------------------
// 座標系定数
// -----------------------------------------------------------------------

// link0_1 (アーム付け根) の base_footprint/world 座標系での高さ [m]
// amir_mecanum3.xacro の base_fixed ジョイント: xyz="0.0 0.0 0.413"
const double LINK0_HEIGHT = 0.413;

// link0_1 から見たテーブル面までの距離 (下方向が正) [m]
// ※ 旧コードから変更なし。衝突オブジェクトを link0_1 フレームで定義するため不要だが
//    moveTo の z 計算に使用する。
const double BASE_HEIGHT = 0.248;

// 物体の大きさ (x, y, z) [m]
const std::vector<double> OBJECT_DIMENSION = {0.075, 0.03, 0.13};

// 物体の link0_1 座標系での位置 (x, y, z) [m]
// ※ 衝突オブジェクトの frame_id = "link0_1" に合わせた値 (旧コードから変更なし)
const std::vector<double> OBJECT_POSITION_LOCAL = {
  0.4,
  -0.018171 + OBJECT_DIMENSION[1] / 2.0,   // ≈ -0.003171
  -BASE_HEIGHT + OBJECT_DIMENSION[2] / 2.0  // ≈ -0.183
};

// 物体の world (base_footprint) 座標系での位置 (x, y, z) [m]
// moveTo に渡す座標は world フレームなので LINK0_HEIGHT を加算する
const std::vector<double> OBJECT_POSITION_WORLD = {
  OBJECT_POSITION_LOCAL[0],
  OBJECT_POSITION_LOCAL[1],
  OBJECT_POSITION_LOCAL[2] + LINK0_HEIGHT   // link0_1 ローカル z + アーム台高さ
};

// -----------------------------------------------------------------------
// ユーティリティ関数
// -----------------------------------------------------------------------

inline double deg2rad(double deg) {
  return deg * M_PI / 180.0;
}

// シーンに衝突オブジェクトを追加
// frame_id = "link0_1" のまま使用するため、座標値は旧コードと同じ
void addCollisionObjects(PlanningSceneInterface &scene) {
  std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
  collision_objects.resize(1);  // 修正: 使用する1つだけ確保

  // 物体を置く小テーブル (link0_1 フレーム基準)
  collision_objects[0].id = "table2";
  collision_objects[0].header.frame_id = "link0_1";  // ← link0_1 フレームのまま
  shape_msgs::msg::SolidPrimitive table2_primitive;
  table2_primitive.type = table2_primitive.BOX;
  table2_primitive.dimensions.resize(3);
  table2_primitive.dimensions[0] = OBJECT_DIMENSION[0];
  table2_primitive.dimensions[1] = OBJECT_DIMENSION[1];
  table2_primitive.dimensions[2] = OBJECT_DIMENSION[2];
  geometry_msgs::msg::Pose table2_pose;
  table2_pose.orientation.w = 1.0;
  table2_pose.position.x = OBJECT_POSITION_LOCAL[0];
  table2_pose.position.y = OBJECT_POSITION_LOCAL[1];
  table2_pose.position.z = -BASE_HEIGHT - 0.005;  // テーブル面より少し下
  collision_objects[0].primitives.push_back(table2_primitive);
  collision_objects[0].primitive_poses.push_back(table2_pose);
  collision_objects[0].operation = collision_objects[0].ADD;

  scene.applyCollisionObjects(collision_objects);
}

// 物体をアタッチ
void attachObject(MoveGroupInterface &group) {
  group.attachObject("object", "gripper_base_1");
  rclcpp::sleep_for(1s);
}

// 物体をデタッチ
void detachObject(MoveGroupInterface &group) {
  group.detachObject("object");
  rclcpp::sleep_for(1s);
}

// グリッパーを開閉
void controlGripper(MoveGroupInterface &group, double angle_deg) {
  std::vector<double> joint_positions(6, deg2rad(angle_deg));
  group.setJointValueTarget(joint_positions);
  group.setGoalTolerance(0.1);
  auto result = group.move();
  if (result != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
    RCLCPP_WARN(rclcpp::get_logger("pick_place"), "Gripper control failed");
  }
}

// 指定姿勢へ移動
// 座標は world (base_footprint) フレーム基準
void moveTo(MoveGroupInterface &group,
            double x, double y, double z,
            double roll_deg, double pitch_deg, double yaw_deg,
            double speed_scaling) {
  tf2::Quaternion q;
  q.setRPY(deg2rad(roll_deg), deg2rad(pitch_deg), deg2rad(yaw_deg));
  geometry_msgs::msg::Pose target;
  target.orientation = tf2::toMsg(q);
  target.position.x = x;
  target.position.y = y;
  target.position.z = z;
  group.setMaxVelocityScalingFactor(speed_scaling);
  group.setPoseTarget(target);
  group.setGoalTolerance(0.01);
  auto result = group.move();
  if (result != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
    RCLCPP_WARN(rclcpp::get_logger("pick_place"), "Motion to target failed");
  }
}

// -----------------------------------------------------------------------
// メイン
// -----------------------------------------------------------------------

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("pick_place");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  MoveGroupInterface arm(node, "arm");
  MoveGroupInterface gripper(node, "gripper");
  PlanningSceneInterface planning_scene_interface;

  arm.setPlanningTime(30.0);
  gripper.setPlanningTime(10.0);

  // -----------------------------------------------------------------------
  // 以下の moveTo の z 座標は world (base_footprint) フレーム基準。
  // 旧コードの z に対して LINK0_HEIGHT (0.413) を加算している。
  //
  // 旧: moveTo(arm, x, y, z_old, ...)
  // 新: moveTo(arm, x, y, z_old + LINK0_HEIGHT, ...)
  //
  // OBJECT_POSITION_WORLD[2] = OBJECT_POSITION_LOCAL[2] + LINK0_HEIGHT
  // -----------------------------------------------------------------------

  rclcpp::sleep_for(1s);
  // addCollisionObjects(planning_scene_interface);

  // pick 操作
  RCLCPP_INFO(node->get_logger(), "Move above object");
  moveTo(arm,
    OBJECT_POSITION_WORLD[0],
    OBJECT_POSITION_WORLD[1],
    OBJECT_DIMENSION[2] + 0.05 + LINK0_HEIGHT,  // 旧: OBJECT_DIMENSION[2] + 0.05
    -90.0, 0.0, -90.0, 0.6);

  controlGripper(gripper, -60.0);

  RCLCPP_INFO(node->get_logger(), "Approach object slowly");
  moveTo(arm,
    OBJECT_POSITION_WORLD[0],
    OBJECT_POSITION_WORLD[1],
    OBJECT_DIMENSION[2] - 0.05 + LINK0_HEIGHT,  // 旧: OBJECT_DIMENSION[2] - 0.05
    -90.0, 0.0, -90.0, 0.3);

  controlGripper(gripper, -2.0);
  // attachObject(arm);

  RCLCPP_INFO(node->get_logger(), "Lift object");
  moveTo(arm,
    OBJECT_POSITION_WORLD[0],
    OBJECT_POSITION_WORLD[1],
    OBJECT_DIMENSION[2] + 0.1 + LINK0_HEIGHT,   // 旧: OBJECT_DIMENSION[2] + 0.1
    -90.0, 0.0, -90.0, 0.3);

  // 中間姿勢 (world フレーム)
  moveTo(arm, 0.0, 0.4, 0.4 + LINK0_HEIGHT, -90.0, 0.0, -90.0, 0.6);

  // place 操作
  RCLCPP_INFO(node->get_logger(), "Move to place location");
  moveTo(arm,
    -OBJECT_POSITION_WORLD[0],
    OBJECT_POSITION_WORLD[1],
    OBJECT_DIMENSION[2] + 0.4 + LINK0_HEIGHT,   // 旧: OBJECT_DIMENSION[2] + 0.4
    -90.0, 0.0, 90.0, 0.6);

  RCLCPP_INFO(node->get_logger(), "Place object slowly");
  moveTo(arm,
    -OBJECT_POSITION_WORLD[0],
    OBJECT_POSITION_WORLD[1],
    OBJECT_DIMENSION[2] + 0.18 + LINK0_HEIGHT,  // 旧: OBJECT_DIMENSION[2] + 0.18
    -90.0, 0.0, 90.0, 0.3);

  controlGripper(gripper, -60.0);
  detachObject(arm);

  RCLCPP_INFO(node->get_logger(), "Retreat");
  moveTo(arm,
    -OBJECT_POSITION_WORLD[0],
    OBJECT_POSITION_WORLD[1],
    OBJECT_DIMENSION[2] + 0.30 + LINK0_HEIGHT,  // 旧: OBJECT_DIMENSION[2] + 0.30
    -90.0, 0.0, 90.0, 0.3);

  RCLCPP_INFO(node->get_logger(), "Return to home");
  moveTo(arm, 0.0, 0.4, 0.4 + LINK0_HEIGHT, 0.0, 0.0, 0.0, 0.6);

  controlGripper(gripper, -20.0);

  RCLCPP_INFO(node->get_logger(), "Pick and place done");

  executor.spin_some();
  rclcpp::shutdown();
  return 0;
}
