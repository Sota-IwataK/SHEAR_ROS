/*
 * ROS 2 Humble / MoveIt2 向け Pick and Place
 * amir_mecanum3 (モバイルマニピュレーター) 対応版
 *
 * ── グラスプモード ──────────────────────────────────────────────
 *   SIDE : グリッパーを地面と平行にして横方向からアプローチ
 *          tcp_link Z → +X (ロボット前方), RPY=(-90°, 0°, -90°)
 *
 *   TOP  : グリッパーを地面と垂直にして真上からアプローチ
 *          tcp_link Z → -Z (鉛直下方),  RPY=(180°, 0°,   0°)
 *
 * GRASP_MODE を変更するだけで切り替え可能。
 *
 * ── 座標系 ──────────────────────────────────────────────────────
 *   全ての目標姿勢は base_footprint フレーム基準。
 *   +X = ロボット前方  +Y = ロボット左方  +Z = 鉛直上方
 *
 * ── グリッパー角度 ───────────────────────────────────────────────
 *   GRIPPER_OPEN  = -1.047 rad (≈ -60°) : 全開
 *   GRIPPER_CLOSE =  0.0   rad          : 閉
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/joint_constraint.hpp>
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
// Joint_4 固定モード
//   true  : Joint_4 を FIXED_JOINT_4_VALUE に固定して位置のみでIK計算
//   false : RPY姿勢を指定して従来の setPoseTarget() でIK計算
// ═══════════════════════════════════════════════════════════════
const bool   USE_JOINT_FIX       = true;
// const double FIXED_JOINT_4_VALUE = -0.2;
const double FIXED_JOINT_4_VALUE = -1.0;      

// ═══════════════════════════════════════════════════════════════
// グラスプモードの選択  ← ここを変更して切り替え
// ═══════════════════════════════════════════════════════════════
enum class GraspMode { SIDE, TOP };
// const GraspMode GRASP_MODE = GraspMode::SIDE;
const GraspMode GRASP_MODE = GraspMode::TOP;

// ── SIDE モード: 水平把持 ────────────────────────────────────────
// tcp_link Z → +X (ロボット前方向き)
// 物体の正面から水平にアプローチし、Y方向に指が開閉する
// const double SIDE_ROLL  = -90.0;
// const double SIDE_PITCH =   0.0;
// const double SIDE_YAW   = -90.0;

const double SIDE_ROLL  =   0.0;
const double SIDE_PITCH = 135.0;
const double SIDE_YAW   =   0.0;

// ── TOP モード: 斜め上からの把持 ──────────────────────────────────
// RPY=(180°,0°,0°) の tcp_Z=-Z (真下向き) は、このアームでは
// x=0.4m 前方では幾何学的に不可能 (J2+J3+J4=180° 時の最大前方リーチ≈0.23m)。
//
// 代替: RPY=(0°, 135°, 0°) → tcp_Z=(+0.707, 0, -0.707)
//   前方 45° 斜め下からアプローチ。J2+J3+J4=-135° で x=0.4m に到達可能。
// const double TOP_ROLL  =   0.0;
// const double TOP_PITCH = 135.0;
// const double TOP_YAW   =   0.0;

const double TOP_ROLL  =   -90.0;
const double TOP_PITCH = 0.0;
const double TOP_YAW   =   -90.0;

// ═══════════════════════════════════════════════════════════════
// 物体の設定 (base_footprint フレーム基準)
// ═══════════════════════════════════════════════════════════════
// 形状: CYLINDER 推奨 (自己センタリング → 把持が安定)
enum class ObjShape { CYLINDER, BOX };
const ObjShape OBJ_SHAPE = ObjShape::CYLINDER;

// ── 円柱パラメータ (CYLINDER 時) ─────────────────────────────────
const double OBJ_RADIUS = 0.025;  // 半径 [m] (diameter=5cm)
const double OBJ_HEIGHT = 0.10;   // 高さ [m]

// ── 直方体パラメータ (BOX 時) ───────────────────────────────────
const double OBJ_SIZE_X = 0.05;
const double OBJ_SIZE_Y = 0.05;
const double OBJ_SIZE_Z = 0.10;

const double OBJ_X = 0.4;   // spawn_box.launch.py のデフォルト x=0.4 に合わせる
const double OBJ_Y = 0.0;
// 地面(z=0)に置いた物体の中心高さ: 高さ/2
const double OBJ_Z = (OBJ_SHAPE == ObjShape::CYLINDER)
                     ? OBJ_HEIGHT / 2.0
                     : OBJ_SIZE_Z / 2.0;

// ── アプローチパラメータ ─────────────────────────────────────────
const double APPROACH_DIST   = 0.10;  // SIDE: 正面からの距離 [m]
const double APPROACH_HEIGHT = 0.15;  // TOP:  真上からの高さ [m]
const double LIFT_HEIGHT     = 0.15;  // 把持後の持ち上げ量 [m]

// ── TOP モード: グリッパー奥(パーム)と物体の隙間調整 ────────────────
// 問題: grip_z = OBJ_Z(物体中心) まで降下するとグリッパーパームが
//       物体上端に当たる。tcp_fixed オフセット 0.0795m があるため
//       TCP より上のパーム構造が物体に干渉しやすい。
// 修正: grip_z を OBJ_Z より TOP_GRIP_Z_OFFSET だけ高くして、
//       パームと物体の間に隙間を確保する。
// 値の目安:
//   0.00: 物体中心まで降下 (パームが干渉しやすい)
//   0.02: 中心+2cm (小さな余裕)
//   0.03: 中心+3cm (推奨デフォルト)
//   0.04: 中心+4cm (余裕大: 指先が物体中心より上になる)
const double TOP_GRIP_Z_OFFSET = 0.03;  // [m]  ← ここを調整して隙間を変える

// ═══════════════════════════════════════════════════════════════
// 配置位置の設定 (base_footprint フレーム基準)
// ═══════════════════════════════════════════════════════════════
const double PLACE_X = 0.4;
const double PLACE_Y = 0.3;        // ロボット左方向 30cm
const double PLACE_Z = 0.15;

// ═══════════════════════════════════════════════════════════════
// グリッパー角度・閉じ挙動パラメータ
// ═══════════════════════════════════════════════════════════════
const double GRIPPER_OPEN  = -1.0;   // 全開 (joint lower limit ≈ -60°)

// 閉じ目標: 物体に必ず接触できるよう、接触角より十分先を指定する。
//
// URDF 解析による指ギャップ概算:
//   gap[m] ≈ 0.046 - 0.08 * sin(θ)
//   θ = -1.00 rad (OPEN)    → gap ≈ 113mm
//   θ = -0.10 rad (旧CLOSE) → gap ≈  54mm  ← 直径5cm物体に届かない!
//   θ =  0.00 rad           → gap ≈  46mm
//   θ = +0.10 rad           → gap ≈  38mm  ← 直径5cm物体に確実に接触
//
// 接触は物理シミュレーションが止める。この値は「接触後も押し続ける」上限。
const double GRIPPER_CLOSE = 0.10;   // [rad]  ← 物体接触角より先に設定

// ── 段階的閉じパラメータ ─────────────────────────────────────────
// STEPS × STEP_MS = 合計閉じ時間  例: 25 × 70ms = 1.75 秒
const int    GRIPPER_CLOSE_STEPS   = 25;    // 分割ステップ数
const int    GRIPPER_CLOSE_STEP_MS = 70;    // ステップ間隔 [ms]

// 接触検知: 速度ではなく「指令位置 vs 実際位置の誤差」で判定
// 段階閉じ中は速度が自然に低いため速度ベース検知は誤作動しやすい
// 指令したのに指が動かない(誤差が閾値以上) = 物体に当たって止まっている
const double GRIPPER_CONTACT_THRESHOLD = 0.02;  // [rad] 位置誤差の閾値

// 接触検知後の保持目標 [rad]
// PRELOAD=0: 接触点ぴったりで保持 → 食い込みゼロ → kp×0=力ゼロ → 振動しない
// 把持力は「指が接触点で物体の動きを機械的に拘束する摩擦力」で生まれる
// (物体が重力で動こうとすると指に押し付けられる → 法線力 × mu ≥ 重力)
const double GRIPPER_PRELOAD = 0.0;   // [rad]  振動ゼロ: 接触境界で止める

// ───────────────────────────────────────────────────────────────

inline double deg2rad(double deg) { return deg * M_PI / 180.0; }

// ───────────────────────────────────────────────────────────────
// グリッパー制御 (open / 位置指定 close 共用)
// ───────────────────────────────────────────────────────────────
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

// ───────────────────────────────────────────────────────────────
// 段階的グリッパー閉鎖 + 位置誤差ベース接触検知
//
// 一気に閉じず STEPS × STEP_MS かけてゆっくり閉じる。
// 各ステップで「指令位置 - 実際位置」の誤差を監視し、
// 誤差が GRIPPER_CONTACT_THRESHOLD を超えたら接触と判定する。
//
// なぜ速度でなく位置誤差か:
//   段階閉じ中はステップ間で速度が自然に低下するため速度ベースは誤検知しやすい。
//   位置誤差は「コマンドを出したのに実際に動かない」ことを直接検知できる。
//
// PRELOAD の役割:
//   接触位置より先を目標にすることでコントローラが物体を押し続け = 把持力
// ───────────────────────────────────────────────────────────────
void closeGripperGradually(
  rclcpp::Node::SharedPtr node,
  GripperClient::SharedPtr client,
  double target_pos = GRIPPER_CLOSE)
{
  using JointState = sensor_msgs::msg::JointState;

  if (!client->wait_for_action_server(5s)) {
    RCLCPP_ERROR(node->get_logger(), "Gripper action server not available!");
    return;
  }

  // /joint_states から Gripper 関節位置を取得
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

  rclcpp::sleep_for(200ms);          // 最初の joint_states が届くまで待機
  double start_pos = g_pos.load();   // 実際の現在位置を取得

  RCLCPP_INFO(node->get_logger(),
    "Gripper 段階的閉じ: %.3f → %.3f rad  (%d steps × %dms = %.1fs)",
    start_pos, target_pos,
    GRIPPER_CLOSE_STEPS, GRIPPER_CLOSE_STEP_MS,
    GRIPPER_CLOSE_STEPS * GRIPPER_CLOSE_STEP_MS / 1000.0);

  double range       = target_pos - start_pos;
  bool   contacted   = false;
  double contact_pos = target_pos;

  for (int step = 1; step <= GRIPPER_CLOSE_STEPS && rclcpp::ok(); ++step) {
    double cmd_pos = start_pos + range * static_cast<double>(step) / GRIPPER_CLOSE_STEPS;

    auto goal = GripperCommand::Goal();
    goal.command.position   = cmd_pos;
    goal.command.max_effort = 50.0;
    client->async_send_goal(goal);   // fire-and-forget (次ステップで上書き可)

    rclcpp::sleep_for(std::chrono::milliseconds(GRIPPER_CLOSE_STEP_MS));

    double actual = g_pos.load();
    // 位置誤差 = 指令値 - 実際値 (正値 = 「もっと閉じろ」と言ったのに動いていない)
    double error  = cmd_pos - actual;

    RCLCPP_DEBUG(node->get_logger(),
      "  step %2d/%d  cmd=%.3f  actual=%.3f  error=%.3f",
      step, GRIPPER_CLOSE_STEPS, cmd_pos, actual, error);

    // 最初の 4 ステップは動き始め猶予として無視
    // error > CONTACT_THRESHOLD かつ閉じ方向に指令が出ている = 接触
    if (step >= 4 && error > GRIPPER_CONTACT_THRESHOLD) {
      contact_pos = actual;
      contacted   = true;
      RCLCPP_INFO(node->get_logger(),
        "Gripper 接触検知! step=%d  cmd=%.3f  actual=%.3f  error=%.3f rad",
        step, cmd_pos, actual, error);
      break;
    }
  }

  if (!contacted) {
    RCLCPP_WARN(node->get_logger(),
      "Gripper 接触未検知 (全 %d ステップ完了)。物体位置を確認してください。",
      GRIPPER_CLOSE_STEPS);
  }

  // ── ホールド: 接触位置 + プリロードで把持力を生成 ──────────────
  double hold_target = contacted
    ? std::min(contact_pos + GRIPPER_PRELOAD, 0.261799)
    : target_pos;

  auto hold = GripperCommand::Goal();
  hold.command.position   = hold_target;
  hold.command.max_effort = 50.0;
  auto gh = client->async_send_goal(hold).get();
  if (gh) {
    auto res = client->async_get_result(gh).get();
    RCLCPP_INFO(node->get_logger(),
      "Gripper ホールド: target=%.3f  actual=%.3f  reached=%s  stalled=%s",
      hold_target, res.result->position,
      res.result->reached_goal ? "yes" : "no",
      res.result->stalled      ? "yes ← 把持力確認" : "no");
  }
}

// ───────────────────────────────────────────────────────────────
// アームを base_footprint フレーム基準の姿勢へ移動
//
// USE_JOINT_FIX == true の場合:
//   Joint_4 を FIXED_JOINT_4_VALUE に固定するパス拘束を設定し、
//   位置のみ (setPositionTarget) で IK を解く。
//   RPY 引数は無視される (ログ表示のみ使用)。
//
// USE_JOINT_FIX == false の場合:
//   従来通り setPoseTarget() で位置 + 姿勢を指定する。
//   ori_tol: 姿勢許容誤差 [rad] (省略時 M_PI/4 = 45°)
// ───────────────────────────────────────────────────────────────
bool moveTo(MoveGroupInterface &group,
            double x, double y, double z,
            double roll_deg, double pitch_deg, double yaw_deg,
            double speed_scaling,
            double ori_tol = M_PI / 4.0,
            bool apply_joint_constraint = true)
{
  group.setStartStateToCurrentState();
  group.setMaxVelocityScalingFactor(speed_scaling);
  group.setMaxAccelerationScalingFactor(speed_scaling * 0.5);
  group.setGoalPositionTolerance(0.01);

  if (USE_JOINT_FIX && apply_joint_constraint) {
    // Joint_4 固定拘束
    moveit_msgs::msg::Constraints constraints;
    moveit_msgs::msg::JointConstraint jc;
    jc.joint_name    = "Joint_4";
    jc.position      = FIXED_JOINT_4_VALUE;
    jc.tolerance_above = 0.01;
    jc.tolerance_below = 0.01;
    jc.weight        = 1.0;
    constraints.joint_constraints.push_back(jc);
    group.setPathConstraints(constraints);

    group.setPositionTarget(x, y, z);

    RCLCPP_INFO(rclcpp::get_logger("pick_place"),
      "[JointFix] Moving to (%.3f, %.3f, %.3f)  Joint_4=%.2f rad  speed=%.2f",
      x, y, z, FIXED_JOINT_4_VALUE, speed_scaling);
  } else if (USE_JOINT_FIX && !apply_joint_constraint) {
    // 拘束なし: 位置のみ指定、姿勢は自由 (PLACE など遠方移動向け)
    group.setPositionTarget(x, y, z);

    RCLCPP_INFO(rclcpp::get_logger("pick_place"),
      "[Free] Moving to (%.3f, %.3f, %.3f)  no Joint_4 constraint  speed=%.2f",
      x, y, z, speed_scaling);
  } else {
    tf2::Quaternion q;
    q.setRPY(deg2rad(roll_deg), deg2rad(pitch_deg), deg2rad(yaw_deg));
    geometry_msgs::msg::Pose target;
    target.orientation = tf2::toMsg(q);
    target.position.x  = x;
    target.position.y  = y;
    target.position.z  = z;

    group.setPoseTarget(target);
    group.setGoalOrientationTolerance(ori_tol);

    RCLCPP_INFO(rclcpp::get_logger("pick_place"),
      "Moving to (%.3f, %.3f, %.3f) RPY=(%.1f, %.1f, %.1f) ori_tol=%.0f° speed=%.2f",
      x, y, z, roll_deg, pitch_deg, yaw_deg, ori_tol * 180.0 / M_PI, speed_scaling);
  }

  auto result = group.move();

  if (USE_JOINT_FIX) {
    group.clearPathConstraints();
  }

  if (result != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("pick_place"),
      "Motion to (%.3f, %.3f, %.3f) FAILED (code: %d) → シーケンス中断", x, y, z, result.val);
    return false;
  }
  return true;
}

// ───────────────────────────────────────────────────────────────
// 現在姿勢を維持したまま指定座標へ直線移動 (Cartesian path)
//
// Joint_4 パス拘束方式の問題:
//   Free 移動後の Joint_4 は不定値になる。その状態から
//   tolerance=±0.01 rad の拘束を設けると start state が
//   拘束を満たさず即座に PLANNING_FAILED (code=-2) する。
//
// 解決策: computeCartesianPath で TCP を直線補間する。
//   - 短距離降下(アプローチ/配置)に最適
//   - 関節拘束なしで姿勢を自然に維持できる
//   - fraction < 0.95 なら失敗を返す
// ───────────────────────────────────────────────────────────────
bool moveCartesian(MoveGroupInterface &group,
                   double x, double y, double z,
                   double speed_scaling)
{
  group.setStartStateToCurrentState();

  // 現在の EEF 姿勢を取得し、位置だけ書き換えてウェイポイントにする
  geometry_msgs::msg::Pose target = group.getCurrentPose().pose;
  target.position.x = x;
  target.position.y = y;
  target.position.z = z;

  std::vector<geometry_msgs::msg::Pose> waypoints = {target};
  moveit_msgs::msg::RobotTrajectory traj_msg;
  // eef_step=5mm, jump_threshold=0 (無効化)
  double fraction = group.computeCartesianPath(waypoints, 0.005, 0.0, traj_msg);

  RCLCPP_INFO(rclcpp::get_logger("pick_place"),
    "[Cartesian] Moving to (%.3f, %.3f, %.3f)  coverage=%.0f%%  speed=%.2f",
    x, y, z, fraction * 100.0, speed_scaling);

  if (fraction < 0.95) {
    RCLCPP_ERROR(rclcpp::get_logger("pick_place"),
      "Cartesian path planning failed: %.0f%% achieved (need ≥95%%)", fraction * 100.0);
    return false;
  }

  // computeCartesianPath はスピードスケーリングを無視するため再パラメータ化
  robot_trajectory::RobotTrajectory rt(group.getRobotModel(), group.getName());
  rt.setRobotTrajectoryMsg(*group.getCurrentState(), traj_msg);
  trajectory_processing::TimeOptimalTrajectoryGeneration totg;
  totg.computeTimeStamps(rt, speed_scaling, speed_scaling * 0.5);
  rt.getRobotTrajectoryMsg(traj_msg);

  auto result = group.execute(traj_msg);
  if (result != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("pick_place"),
      "Cartesian execute FAILED (code: %d)", result.val);
    return false;
  }
  return true;
}

// ───────────────────────────────────────────────────────────────
// 衝突オブジェクトをシーンに追加 (OBJ_SHAPE に応じて形状を切り替え)
// ───────────────────────────────────────────────────────────────
void addCollisionObjects(PlanningSceneInterface &scene)
{
  moveit_msgs::msg::CollisionObject obj;
  obj.id              = "pick_object";
  obj.header.frame_id = "base_footprint";

  shape_msgs::msg::SolidPrimitive prim;

  if (OBJ_SHAPE == ObjShape::CYLINDER) {
    // CYLINDER: dimensions[0]=height, dimensions[1]=radius
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

  RCLCPP_INFO(node->get_logger(), "Planning frame : %s", arm.getPlanningFrame().c_str());
  RCLCPP_INFO(node->get_logger(), "End effector   : %s", arm.getEndEffectorLink().c_str());
  RCLCPP_INFO(node->get_logger(), "Grasp mode     : %s",
    GRASP_MODE == GraspMode::SIDE ? "SIDE (水平把持)" : "TOP (垂直把持)");
  RCLCPP_INFO(node->get_logger(),
    "Object center  : x=%.3f y=%.3f z=%.3f", OBJ_X, OBJ_Y, OBJ_Z);

  // ── モードごとの姿勢・位置パラメータを確定 ─────────────────────
  double g_roll, g_pitch, g_yaw;
  // 把持時の姿勢許容誤差: 両モードとも 30° を上限とし、物体把持の方向性を確保する
  const double ORI_TOL = M_PI / 6.0;   // 30°

  double pre_x,  pre_y,  pre_z;   // アプローチ準備位置
  double grip_x, grip_y, grip_z;  // 実際の把持位置
  double lift_z;                   // 持ち上げ後の高さ

  if (GRASP_MODE == GraspMode::SIDE) {
    // ── 水平把持 ─────────────────────────────────────────────────
    // tcp_link Z が +X を向く → 物体の正面から水平アプローチ
    //
    // アームベースは z=0.413m にある。低い z=OBJ_Z(≈0.05m) に水平姿勢で
    // 直接移動しようとすると関節限界でプランニング失敗する。
    // そのため: ① 高め準備位置 → ② グリップ高さに降下 → ③ 前進把持
    // という 2 段アプローチを使う。
    g_roll = SIDE_ROLL; g_pitch = SIDE_PITCH; g_yaw = SIDE_YAW;
    // 準備位置: 前方 APPROACH_DIST 手前、obj上端 + APPROACH_HEIGHT の高さ
    double obj_top_side = OBJ_Z + ((OBJ_SHAPE == ObjShape::CYLINDER)
                                    ? OBJ_HEIGHT / 2.0 : OBJ_SIZE_Z / 2.0);
    pre_x  = OBJ_X - APPROACH_DIST;
    pre_y  = OBJ_Y;
    pre_z  = obj_top_side + APPROACH_HEIGHT;  // ← 高い位置 (旧: OBJ_Z)
    grip_x = OBJ_X;                 grip_y = OBJ_Y; grip_z = OBJ_Z;
    lift_z = OBJ_Z + LIFT_HEIGHT;
  } else {
    // ── 垂直把持 ─────────────────────────────────────────────────
    // 真上から降下し、フィンガーで両側から挟んで把持する
    g_roll = TOP_ROLL; g_pitch = TOP_PITCH; g_yaw = TOP_YAW;
    // 物体上端 (円柱/直方体どちらにも対応)
    double obj_top = OBJ_Z + ((OBJ_SHAPE == ObjShape::CYLINDER)
                               ? OBJ_HEIGHT / 2.0
                               : OBJ_SIZE_Z / 2.0);
    // grip_z: OBJ_Z より TOP_GRIP_Z_OFFSET 分高く設定してパームとの干渉を回避
    // 指先は OBJ_Z 付近に来るため把持は可能
    pre_x  = OBJ_X; pre_y  = OBJ_Y; pre_z  = obj_top + APPROACH_HEIGHT;
    grip_x = OBJ_X; grip_y = OBJ_Y; grip_z = OBJ_Z + TOP_GRIP_Z_OFFSET;
    lift_z = obj_top + LIFT_HEIGHT;
    RCLCPP_INFO(rclcpp::get_logger("pick_place"),
      "TOP grip_z=%.3f (OBJ_Z=%.3f + offset=%.3f)", grip_z, OBJ_Z, TOP_GRIP_Z_OFFSET);
  }

  addCollisionObjects(planning_scene_interface);
  rclcpp::sleep_for(1s);

  // ═══════════════════════════════════════
  // PICK シーケンス
  // ═══════════════════════════════════════
  RCLCPP_INFO(node->get_logger(), "════ PICK ════");

  RCLCPP_INFO(node->get_logger(), "── グリッパーを開く");
  controlGripper(node, gripper_client, GRIPPER_OPEN);

  RCLCPP_INFO(node->get_logger(), "── アプローチ準備位置へ移動 (高め位置)");
  if (!moveTo(arm, pre_x, pre_y, pre_z, g_roll, g_pitch, g_yaw, 0.6, ORI_TOL, false)) {
    RCLCPP_ERROR(node->get_logger(), "準備位置への移動失敗。中断します。");
    rclcpp::shutdown(); spinner.join(); return 1;
  }

  // SIDE モード: 準備位置 (高め) からグリップ高さまで降下してから前進
  // アームが直接低い z へ水平姿勢で移動できない場合の 2 段アプローチ
  if (GRASP_MODE == GraspMode::SIDE) {
    RCLCPP_INFO(node->get_logger(), "── [SIDE] グリップ高さまで降下");
    if (!moveTo(arm, pre_x, pre_y, grip_z, g_roll, g_pitch, g_yaw, 0.3, ORI_TOL, false)) {
      RCLCPP_ERROR(node->get_logger(), "グリップ高さへの降下失敗。中断します。");
      rclcpp::shutdown(); spinner.join(); return 1;
    }
  }

  // pick_object は準備位置までの経路計画（回避）に使用した。
  // 把持動作でグリッパーが物体内部に入るため、直前に除去して経路を開放する。
  RCLCPP_INFO(node->get_logger(), "── コリジョンオブジェクト除去 (把持のため経路開放)");
  planning_scene_interface.removeCollisionObjects({"pick_object"});
  rclcpp::sleep_for(500ms);

  RCLCPP_INFO(node->get_logger(), "── 把持位置へゆっくり降下 [Cartesian 直線補間]");
  if (!moveCartesian(arm, grip_x, grip_y, grip_z, 0.3)) {
    RCLCPP_ERROR(node->get_logger(), "把持位置への移動失敗。中断します。");
    rclcpp::shutdown(); spinner.join(); return 1;
  }

  // 段階的グリッパー閉鎖 + 接触検知
  RCLCPP_INFO(node->get_logger(), "── グリッパーを段階的に閉じる");
  closeGripperGradually(node, gripper_client, GRIPPER_CLOSE);

  RCLCPP_INFO(node->get_logger(), "── 物体を持ち上げる");
  if (!moveTo(arm, OBJ_X, OBJ_Y, lift_z, g_roll, g_pitch, g_yaw, 0.3, ORI_TOL, false)) {
    RCLCPP_ERROR(node->get_logger(), "持ち上げ失敗。中断します。");
    rclcpp::shutdown(); spinner.join(); return 1;
  }

  // ═══════════════════════════════════════
  // PLACE シーケンス
  // ═══════════════════════════════════════
  RCLCPP_INFO(node->get_logger(), "════ PLACE ════");

  RCLCPP_INFO(node->get_logger(), "── 配置位置の上方へ移動");
  if (!moveTo(arm, PLACE_X, PLACE_Y, lift_z, g_roll, g_pitch, g_yaw, 0.6, ORI_TOL, false)) {
    RCLCPP_ERROR(node->get_logger(), "配置位置上方への移動失敗。中断します。");
    rclcpp::shutdown(); spinner.join(); return 1;
  }

  RCLCPP_INFO(node->get_logger(), "── 配置位置へゆっくり降下 [Cartesian 直線補間]");
  if (!moveCartesian(arm, PLACE_X, PLACE_Y, PLACE_Z, 0.3)) {
    RCLCPP_ERROR(node->get_logger(), "配置位置への降下失敗。中断します。");
    rclcpp::shutdown(); spinner.join(); return 1;
  }

  RCLCPP_INFO(node->get_logger(), "── グリッパーを開く (物体を放す)");
  controlGripper(node, gripper_client, GRIPPER_OPEN);

  RCLCPP_INFO(node->get_logger(), "── 退避");
  moveTo(arm, PLACE_X, PLACE_Y, lift_z, g_roll, g_pitch, g_yaw, 0.3, ORI_TOL, false);

  RCLCPP_INFO(node->get_logger(), "════ Pick and place 完了 ════");

  rclcpp::shutdown();
  spinner.join();
  return 0;
}
