/*
 * ROS 2 Humble 向け Pick and Place サンプル (5自由度モバイルマニピュレータ対応版)
 */

 #include <rclcpp/rclcpp.hpp>
 #include <moveit/planning_scene_interface/planning_scene_interface.h>
 #include <moveit/move_group_interface/move_group_interface.h>
 #include <moveit_msgs/msg/collision_object.hpp>
 #include <shape_msgs/msg/solid_primitive.hpp>
 #include <tf2/LinearMath/Quaternion.h>
 #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
 #include <geometry_msgs/msg/pose.hpp>
 #include <chrono>
 #include <vector>
 #include <cmath>
 #include <thread>
 
 using namespace std::chrono_literals;
 using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
 using PlanningSceneInterface = moveit::planning_interface::PlanningSceneInterface;
 
 // ベースからテーブルまでの高さ
 const double BASE_HEIGHT = 0.248;
 // 物体の大きさ (x, y, z)
 const std::vector<double> OBJECT_DIMENSION = {0.075, 0.03, 0.13};
 // 物体のワールド座標系での位置 (x, y, z)
 const std::vector<double> OBJECT_POSITION = {
   0.4,
   -0.018171 + OBJECT_DIMENSION[1] / 2.0,
   -BASE_HEIGHT + OBJECT_DIMENSION[2] / 2.0
 };
 
 // 度をラジアンに変換
 inline double deg2rad(double deg) {
   return deg * M_PI / 180.0;
 }
 
 // シーンにテーブルなどの衝突オブジェクトを追加
 void addCollisionObjects(PlanningSceneInterface &scene) {
   std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
   collision_objects.resize(1);
 
   // 物体を置く小テーブル
   collision_objects[0].id = "table2";
   collision_objects[0].header.frame_id = "base_footprint"; // ルートリンクに合わせる
   shape_msgs::msg::SolidPrimitive table2_primitive;
   table2_primitive.type = table2_primitive.BOX;
   table2_primitive.dimensions.resize(3);
   table2_primitive.dimensions[0] = OBJECT_DIMENSION[0];
   table2_primitive.dimensions[1] = OBJECT_DIMENSION[1];
   table2_primitive.dimensions[2] = OBJECT_DIMENSION[2];
 
   geometry_msgs::msg::Pose table2_pose;
   table2_pose.orientation.w = 1.0;
   table2_pose.position.x = OBJECT_POSITION[0];
   table2_pose.position.y = OBJECT_POSITION[1];
   table2_pose.position.z = -BASE_HEIGHT - 0.005;
 
   collision_objects[0].primitives.push_back(table2_primitive);
   collision_objects[0].primitive_poses.push_back(table2_pose);
   collision_objects[0].operation = collision_objects[0].ADD;
 
   // シーンに適用
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
   // Setup AssistantのActive Joint数に合わせ、サイズを1に設定
   std::vector<double> joint_positions(1, deg2rad(angle_deg));
   group.setJointValueTarget(joint_positions);
   group.setGoalTolerance(0.1);
   auto result = group.move();
   if (result != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
     RCLCPP_WARN(rclcpp::get_logger("pick_place"), "Gripper control failed");
   }
 }
 
 // 指定姿勢へ移動
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
   
   // 5自由度アーム向け：厳密なPoseTargetではなく、近い姿勢（近似解）を探して移動する
   group.setApproximateJointValueTarget(target);
   
   auto result = group.move();
   if (result != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
     RCLCPP_WARN(rclcpp::get_logger("pick_place"), "Motion to target failed");
   }
 }
 
 int main(int argc, char **argv) {
   rclcpp::init(argc, argv);
   
   // パラメータの自動宣言を有効化
   rclcpp::NodeOptions node_options;
   node_options.automatically_declare_parameters_from_overrides(true);
   auto node = rclcpp::Node::make_shared("pick_place", node_options);
 
   // MultiThreadedExecutorを用いて別スレッドでイベントを処理し、デッドロックを防ぐ
   rclcpp::executors::MultiThreadedExecutor executor;
   executor.add_node(node);
   std::thread spinner_thread([&executor]() { executor.spin(); });
 
   MoveGroupInterface arm(node, "arm");
   MoveGroupInterface gripper(node, "gripper");
   PlanningSceneInterface planning_scene_interface;
 
   arm.setPlanningTime(30.0);
   gripper.setPlanningTime(10.0);
 
   rclcpp::sleep_for(1s);
   // addCollisionObjects(planning_scene_interface);
 
   // pick 操作
   RCLCPP_INFO(node->get_logger(), "Move above object");
   moveTo(arm, OBJECT_POSITION[0], OBJECT_POSITION[1], OBJECT_DIMENSION[2] + 0.05, -90.0, 0.0, -90.0, 0.6);
   controlGripper(gripper, -60.0);
   
   moveTo(arm, OBJECT_POSITION[0], OBJECT_POSITION[1], OBJECT_DIMENSION[2] - 0.05, -90.0, 0.0, -90.0, 0.3);
   controlGripper(gripper, -2.0);
 
   RCLCPP_INFO(node->get_logger(), "Lift object");
   moveTo(arm, OBJECT_POSITION[0], OBJECT_POSITION[1], OBJECT_DIMENSION[2] + 0.1, -90.0, 0.0, -90.0, 0.3);
   moveTo(arm, 0.0, 0.4, 0.4, -90.0, 0.0, -90.0, 0.6);
   
   // place 操作
   RCLCPP_INFO(node->get_logger(), "Move to place location");
   moveTo(arm, -OBJECT_POSITION[0], OBJECT_POSITION[1], OBJECT_DIMENSION[2] + 0.4, -90.0, 0.0, 90.0, 0.6);
   
   RCLCPP_INFO(node->get_logger(), "Place object slowly");
   moveTo(arm, -OBJECT_POSITION[0], OBJECT_POSITION[1], OBJECT_DIMENSION[2] + 0.18, -90.0, 0.0, 90.0, 0.3);
 
   controlGripper(gripper, -60.0);
   detachObject(arm);
 
   RCLCPP_INFO(node->get_logger(), "Retreat");
   moveTo(arm, -OBJECT_POSITION[0], OBJECT_POSITION[1], OBJECT_DIMENSION[2] + 0.30, -90.0, 0.0, 90.0, 0.3);
   
   RCLCPP_INFO(node->get_logger(), "Return to home");
   moveTo(arm, 0.0, 0.4, 0.4, 0.0, 0.0, 0.0, 0.6);
   controlGripper(gripper, -20.0);
 
   // 終了処理
   rclcpp::shutdown();
   spinner_thread.join();
   return 0;
 }