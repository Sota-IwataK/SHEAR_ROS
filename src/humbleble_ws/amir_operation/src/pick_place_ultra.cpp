/*
 * ROS 2 Humble / MoveIt2 向け Pick and Place
 * amir_mecanum3 (モバイルマニピュレーター) 対応版
 *
 * ── 座標系と姿勢指定のハイブリッド対応 ──────────────────────────────
 * 位置(X, Y, Z): 全て base_footprint フレーム基準の絶対座標
 * 姿勢(R, P, Y): 
 * - moveTo() : base_footprint 基準の絶対姿勢
 * - moveToWithLocalOrientation() : 現在の tcp_link 基準の相対回転
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <control_msgs/action/gripper_command.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <atomic>
#include <chrono>
#include <cmath>
#include <thread>

using namespace std::chrono_literals;
using MoveGroupInterface     = moveit::planning_interface::MoveGroupInterface;
using PlanningSceneInterface = moveit::planning_interface::PlanningSceneInterface;
using GripperCommand         = control_msgs::action::GripperCommand;
using GripperClient          = rclcpp_action::Client<GripperCommand>;

// ═══════════════════════════════════════════════════════════════
// グラスプモードの選択
// ═══════════════════════════════════════════════════════════════
enum class GraspMode { SIDE, TOP };
const GraspMode GRASP_MODE = GraspMode::TOP;

// ── SIDE モード: 水平把持 (絶対姿勢の定義) ────────────────────────
const double SIDE_ROLL  =   0.0;
const double SIDE_PITCH = 135.0;
const double SIDE_YAW   =   0.0;

// ── TOP モード: 斜め上からの把持 (絶対姿勢の定義) ──────────────────
const double TOP_ROLL  =  -90.0;
const double TOP_PITCH =    0.0;
const double TOP_YAW   =  -90.0;

// ═══════════════════════════════════════════════════════════════
// 物体・配置の設定 (base_footprint フレーム基準)
// ═══════════════════════════════════════════════════════════════
enum class ObjShape { CYLINDER, BOX };
const ObjShape OBJ_SHAPE = ObjShape::CYLINDER;

const double OBJ_RADIUS = 0.025; 
const double OBJ_HEIGHT = 0.10;  
const double OBJ_SIZE_X = 0.05;
const double OBJ_SIZE_Y = 0.05;
const double OBJ_SIZE_Z = 0.10;

const double OBJ_X = 0.4;
const double OBJ_Y = 0.0;
const double OBJ_Z = (OBJ_SHAPE == ObjShape::CYLINDER)
                     ? OBJ_HEIGHT / 2.0
                     : OBJ_SIZE_Z / 2.0;

const double APPROACH_DIST   = 0.10; 
const double APPROACH_HEIGHT = 0.15; 
const double LIFT_HEIGHT     = 0.15; 
const double TOP_GRIP_Z_OFFSET = 0.03; 

const double PLACE_X = 0.4;
const double PLACE_Y = 0.3;        
const double PLACE_Z = 0.15;

// ═══════════════════════════════════════════════════════════════
// グリッパー設定
// ═══════════════════════════════════════════════════════════════
const double GRIPPER_OPEN  = -1.0;   
const double GRIPPER_CLOSE = 0.10;   

const int    GRIPPER_CLOSE_STEPS   = 25;    
const int    GRIPPER_CLOSE_STEP_MS = 70;    
const double GRIPPER_CONTACT_THRESHOLD = 0.02; 
const double GRIPPER_PRELOAD = 0.0;   

// ───────────────────────────────────────────────────────────────

inline double deg2rad(double deg) { return deg * M_PI / 180.0; }

// グリッパー制御 (通常)
void controlGripper(
  rclcpp::Node::SharedPtr node,
  GripperClient::SharedPtr client,
  double position,
  double max_effort = 20.0)
{
  if (!client->wait_for_action_server(5s)) {
    RCLCPP_ERROR(node->get_logger(), "Gripper action server not available!");
    return;
  }
  auto goal = GripperCommand::Goal();
  goal.command.position   = position;
  goal.command.max_effort = max_effort;

  auto gh = client->async_send_goal(goal).get();
  if (!gh) { RCLCPP_ERROR(node->get_logger(), "Gripper goal rejected"); return; }

  auto res = client->async_get_result(gh).get();
  RCLCPP_INFO(node->get_logger(),
    "Gripper → pos=%.3f  reached=%s  stalled=%s",
    res.result->position,
    res.result->reached_goal ? "yes" : "no",
    res.result->stalled      ? "yes (接触)" : "no");
}

// 段階的グリッパー閉鎖 + 接触検知
void closeGripperGradually(
  rclcpp::Node::SharedPtr node,
  GripperClient::SharedPtr client,
  double target_pos = GRIPPER_CLOSE)
{
  using JointState = sensor_msgs::msg::JointState;

  if (!client->wait_for_action_server(5s)) return;

  std::atomic<double> g_pos{GRIPPER_OPEN};
  auto js_sub = node->create_subscription<JointState>(
    "/joint_states", rclcpp::QoS(10),
    [&g_pos](JointState::ConstSharedPtr msg) {
      for (size_t i = 0; i < msg->name.size(); ++i) {
        if (msg->name[i] == "Gripper") {
          if (i < msg->position.size()) g_pos.store(msg->position[i]);
          break;
        }
      }
    });

  rclcpp::sleep_for(200ms);         
  double start_pos = g_pos.load();   

  RCLCPP_INFO(node->get_logger(), "Gripper 段階的閉じ開始...");

  double range       = target_pos - start_pos;
  bool   contacted   = false;
  double contact_pos = target_pos;

  for (int step = 1; step <= GRIPPER_CLOSE_STEPS && rclcpp::ok(); ++step) {
    double cmd_pos = start_pos + range * static_cast<double>(step) / GRIPPER_CLOSE_STEPS;

    auto goal = GripperCommand::Goal();
    goal.command.position   = cmd_pos;
    goal.command.max_effort = 50.0;
    client->async_send_goal(goal);   

    rclcpp::sleep_for(std::chrono::milliseconds(GRIPPER_CLOSE_STEP_MS));

    double actual = g_pos.load();
    double error  = cmd_pos - actual;

    if (step >= 4 && error > GRIPPER_CONTACT_THRESHOLD) {
      contact_pos = actual;
      contacted   = true;
      RCLCPP_INFO(node->get_logger(), "Gripper 接触検知! step=%d actual=%.3f", step, actual);
      break;
    }
  }

  double hold_target = contacted ? std::min(contact_pos + GRIPPER_PRELOAD, 0.261799) : target_pos;
  auto hold = GripperCommand::Goal();
  hold.command.position   = hold_target;
  hold.command.max_effort = 50.0;
  client->async_send_goal(hold).get();
}

// ───────────────────────────────────────────────────────────────
// 【従来関数】絶対座標(base_footprint)基準の位置と姿勢へ移動
// ───────────────────────────────────────────────────────────────
bool moveTo(MoveGroupInterface &group,
            double x, double y, double z,
            double roll_deg, double pitch_deg, double yaw_deg,
            double speed_scaling,
            double ori_tol = M_PI / 4.0)
{
  group.setStartStateToCurrentState();

  tf2::Quaternion q;
  q.setRPY(deg2rad(roll_deg), deg2rad(pitch_deg), deg2rad(yaw_deg));
  geometry_msgs::msg::Pose target;
  target.orientation = tf2::toMsg(q);
  target.position.x  = x;
  target.position.y  = y;
  target.position.z  = z;

  group.setMaxVelocityScalingFactor(speed_scaling);
  group.setMaxAccelerationScalingFactor(speed_scaling * 0.5);
  group.setPoseTarget(target);
  group.setGoalPositionTolerance(0.01);
  group.setGoalOrientationTolerance(ori_tol);

  RCLCPP_INFO(rclcpp::get_logger("pick_place"),
    "Move [Absolute] -> Pos(%.3f, %.3f, %.3f) RPY=(%.1f, %.1f, %.1f)",
    x, y, z, roll_deg, pitch_deg, yaw_deg);

  auto result = group.move();
  if (result != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("pick_place"), "Motion FAILED (code: %d)", result.val);
    return false;
  }
  return true;
}

// ───────────────────────────────────────────────────────────────
// 【新規関数】位置は絶対座標、姿勢は手先(tcp_link)基準の相対回転で移動
// ───────────────────────────────────────────────────────────────
bool moveToWithLocalOrientation(
    MoveGroupInterface &group,
    double world_x, double world_y, double world_z,
    double local_roll_deg, double local_pitch_deg, double local_yaw_deg,
    double speed_scaling,
    double ori_tol = M_PI / 4.0)
{
  group.setStartStateToCurrentState();

  // 1. 現在の手先姿勢を取得 (base_footprint基準)
  auto current_pose = group.getCurrentPose().pose;
  tf2::Quaternion q_current;
  tf2::fromMsg(current_pose.orientation, q_current);

  // 2. 手先ローカルフレーム基準の回転量を作成
  tf2::Quaternion q_local_rot;
  q_local_rot.setRPY(deg2rad(local_roll_deg), deg2rad(local_pitch_deg), deg2rad(local_yaw_deg));

  // 3. クォータニオン合成 (現在姿勢 × ローカル回転) ※右から掛けることでローカル軸回転になる
  tf2::Quaternion q_target = q_current * q_local_rot;
  q_target.normalize();

  // 4. 目標Poseの生成
  geometry_msgs::msg::Pose target;
  target.position.x  = world_x;
  target.position.y  = world_y;
  target.position.z  = world_z;
  target.orientation = tf2::toMsg(q_target);

  group.setMaxVelocityScalingFactor(speed_scaling);
  group.setMaxAccelerationScalingFactor(speed_scaling * 0.5);
  group.setPoseTarget(target);
  group.setGoalPositionTolerance(0.01);
  group.setGoalOrientationTolerance(ori_tol);

  RCLCPP_INFO(rclcpp::get_logger("pick_place"),
    "Move [Local Rot] -> World Pos(%.3f, %.3f, %.3f) Local Rot RPY=(%.1f, %.1f, %.1f)",
    world_x, world_y, world_z, local_roll_deg, local_pitch_deg, local_yaw_deg);

  auto result = group.move();
  if (result != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("pick_place"), "Motion FAILED (code: %d)", result.val);
    return false;
  }
  return true;
}

// 衝突オブジェクト追加
void addCollisionObjects(PlanningSceneInterface &scene)
{
  moveit_msgs::msg::CollisionObject obj;
  obj.id              = "pick_object";
  obj.header.frame_id = "base_footprint";

  shape_msgs::msg::SolidPrimitive prim;
  if (OBJ_SHAPE == ObjShape::CYLINDER) {
    prim.type = prim.CYLINDER;
    prim.dimensions.resize(2);
    prim.dimensions[0] = OBJ_HEIGHT;
    prim.dimensions[1] = OBJ_RADIUS;
  } else {
    prim.type = prim.BOX;
    prim.dimensions.resize(3);
    prim.dimensions[0] = OBJ_SIZE_X;
    prim.dimensions[1] = OBJ_SIZE_Y;
    prim.dimensions[2] = OBJ_SIZE_Z;
  }

  geometry_msgs::msg::Pose pose;
  pose.orientation.w = 1.0;
  pose.position.x    = OBJ_X;
  pose.position.y    = OBJ_Y;
  pose.position.z    = OBJ_Z;

  obj.primitives.push_back(prim);
  obj.primitive_poses.push_back(pose);
  obj.operation = obj.ADD;

  scene.applyCollisionObject(obj);
  rclcpp::sleep_for(500ms);
}

// ───────────────────────────────────────────────────────────────
// メイン
// ───────────────────────────────────────────────────────────────
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  node_options.parameter_overrides({{"use_sim_time", true}});
  auto node = rclcpp::Node::make_shared("pick_place", node_options);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  auto gripper_client = rclcpp_action::create_client<GripperCommand>(
    node, "/gripper_controller/gripper_cmd");

  MoveGroupInterface arm(node, "arm");
  PlanningSceneInterface planning_scene_interface;

  arm.setPoseReferenceFrame("base_footprint");
  arm.setPlanningTime(30.0);
  arm.setNumPlanningAttempts(5);

  double g_roll, g_pitch, g_yaw;
  const double ORI_TOL = M_PI / 6.0;   // 30°

  double pre_x,  pre_y,  pre_z;   
  double grip_x, grip_y, grip_z;  
  double lift_z;                   

  if (GRASP_MODE == GraspMode::SIDE) {
    g_roll = SIDE_ROLL; g_pitch = SIDE_PITCH; g_yaw = SIDE_YAW;
    double obj_top_side = OBJ_Z + ((OBJ_SHAPE == ObjShape::CYLINDER) ? OBJ_HEIGHT / 2.0 : OBJ_SIZE_Z / 2.0);
    pre_x  = OBJ_X - APPROACH_DIST;
    pre_y  = OBJ_Y;
    pre_z  = obj_top_side + APPROACH_HEIGHT; 
    grip_x = OBJ_X; grip_y = OBJ_Y; grip_z = OBJ_Z;
    lift_z = OBJ_Z + LIFT_HEIGHT;
  } else {
    g_roll = TOP_ROLL; g_pitch = TOP_PITCH; g_yaw = TOP_YAW;
    double obj_top = OBJ_Z + ((OBJ_SHAPE == ObjShape::CYLINDER) ? OBJ_HEIGHT / 2.0 : OBJ_SIZE_Z / 2.0);
    pre_x  = OBJ_X; pre_y  = OBJ_Y; pre_z  = obj_top + APPROACH_HEIGHT;
    grip_x = OBJ_X; grip_y = OBJ_Y; grip_z = OBJ_Z + TOP_GRIP_Z_OFFSET;
    lift_z = obj_top + LIFT_HEIGHT;
  }

  addCollisionObjects(planning_scene_interface);
  rclcpp::sleep_for(1s);

  // ═══════════════════════════════════════
  // PICK シーケンス
  // ═══════════════════════════════════════
  RCLCPP_INFO(node->get_logger(), "════ PICK ════");
  controlGripper(node, gripper_client, GRIPPER_OPEN);

  RCLCPP_INFO(node->get_logger(), "── アプローチ準備位置へ移動 (絶対姿勢指定)");
  if (!moveTo(arm, pre_x, pre_y, pre_z, g_roll, g_pitch, g_yaw, 0.6, ORI_TOL)) {
    rclcpp::shutdown(); spinner.join(); return 1;
  }

  if (GRASP_MODE == GraspMode::SIDE) {
    RCLCPP_INFO(node->get_logger(), "── [SIDE] グリップ高さまで降下");
    if (!moveTo(arm, pre_x, pre_y, grip_z, g_roll, g_pitch, g_yaw, 0.3, ORI_TOL)) {
      rclcpp::shutdown(); spinner.join(); return 1;
    }
  }

  planning_scene_interface.removeCollisionObjects({"pick_object"});
  rclcpp::sleep_for(500ms);

  RCLCPP_INFO(node->get_logger(), "── 把持位置へ接近");
  
  // ── 【適用例】絶対姿勢移動とローカル姿勢移動の比較・使い分け ──
  // 通常通り絶対姿勢を維持してアプローチする場合は moveTo を使います。
  // ここでは例として、「現在の手先の向きを維持したまま移動する(回転0,0,0)」か、
  // あるいは「手先軸ベースで特定の角度ひねってアプローチする」アプローチを示します。
  
  // 例: 現在の指先の姿勢をそのまま維持して、絶対座標 (grip_x, grip_y, grip_z) へ進む場合
  if (!moveToWithLocalOrientation(arm, grip_x, grip_y, grip_z, 0.0, 0.0, 0.0, 0.3, ORI_TOL)) {
    rclcpp::shutdown(); spinner.join(); return 1;
  }
  
  /* // ※もし手先を現在の向きからローカルのRoll軸で15度回しながらアプローチしたい場合は以下のように書けます
  // moveToWithLocalOrientation(arm, grip_x, grip_y, grip_z, 15.0, 0.0, 0.0, 0.3, ORI_TOL);
  */

  RCLCPP_INFO(node->get_logger(), "── グリッパーを閉じる");
  closeGripperGradually(node, gripper_client, GRIPPER_CLOSE);

  RCLCPP_INFO(node->get_logger(), "── 物体を持ち上げる (手先の姿勢をキープして真上へ)");
  // 持ち上げ時も、把持した手先のローカル姿勢を崩さずに絶対位置(lift_z)へ移動させる
  if (!moveToWithLocalOrientation(arm, OBJ_X, OBJ_Y, lift_z, 0.0, 0.0, 0.0, 0.3, ORI_TOL)) {
    rclcpp::shutdown(); spinner.join(); return 1;
  }

  // ═══════════════════════════════════════
  // PLACE シーケンス
  // ═══════════════════════════════════════
  RCLCPP_INFO(node->get_logger(), "════ PLACE ════");

  // プレース先へは、手先姿勢を維持したまま移動
  RCLCPP_INFO(node->get_logger(), "── 配置位置の上方へ移動");
  if (!moveToWithLocalOrientation(arm, PLACE_X, PLACE_Y, lift_z, 0.0, 0.0, 0.0, 0.6, ORI_TOL)) {
    rclcpp::shutdown(); spinner.join(); return 1;
  }

  RCLCPP_INFO(node->get_logger(), "── 配置位置へ降下");
  if (!moveToWithLocalOrientation(arm, PLACE_X, PLACE_Y, PLACE_Z, 0.0, 0.0, 0.0, 0.3, ORI_TOL)) {
    rclcpp::shutdown(); spinner.join(); return 1;
  }

  controlGripper(node, gripper_client, GRIPPER_OPEN);

  RCLCPP_INFO(node->get_logger(), "── 退避");
  moveToWithLocalOrientation(arm, PLACE_X, PLACE_Y, lift_z, 0.0, 0.0, 0.0, 0.3, ORI_TOL);

  RCLCPP_INFO(node->get_logger(), "════ Pick and place 完了 ════");

  rclcpp::shutdown();
  spinner.join();
  return 0;
}