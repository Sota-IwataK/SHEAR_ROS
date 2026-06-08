/*
 * survey_ik.cpp
 *
 * 目的:
 *   1. RPY 各軸が物理的にどの方向を意味するかを説明する
 *   2. 関節空間を FK でサンプリングし、指定位置で到達可能な tcp_link 姿勢を列挙する
 *
 * 起動:
 *   ros2 launch amir_operation survey_ik_launch.py [x:=0.4] [y:=0.0] [z:=0.4]
 *
 * FK サンプリングは IK 設定 (position_only_ik) に依存しないため、
 * 5 自由度アームで本当に到達できる姿勢のみを正確に報告する。
 *
 * 関節限界 (amir_for_rover.xacro より):
 *   Joint_1: [-170°, +170°]
 *   Joint_2: [  0°, +135°]
 *   Joint_3: [-160°,   0°]
 *   Joint_4: [-120°,  +75°]
 *   Joint_5: [-158°, +158°]
 */

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <thread>
#include <cmath>
#include <vector>
#include <set>
#include <string>
#include <sstream>
#include <iomanip>
#include <algorithm>

// -----------------------------------------------------------------------
// 調査設定 (起動引数で上書き可能)
// -----------------------------------------------------------------------
// 調査対象の位置 [m]  (base_footprint フレーム)
double TARGET_X = 0.4;
double TARGET_Y = 0.0;
double TARGET_Z = 0.4;

// FK サンプリング分解能 [°] (小さいほど精密, 10° 推奨)
const double JOINT_STEP_DEG = 10.0;

// 「目標位置に近い」とみなす距離閾値 [m]
// 10° ステップでの最大位置誤差 ≒ 0.31 * sin(10°) ≈ 0.054m なので 0.06m に設定
const double POS_THRESH = 0.06;

// RPY の丸め精度 [°] (この単位で重複排除する)
const double RPY_ROUND_DEG = 10.0;

// -----------------------------------------------------------------------

static const rclcpp::Logger LOG = rclcpp::get_logger("survey_ik");

inline double r2d(double r) { return r * 180.0 / M_PI; }
inline double d2r(double d) { return d * M_PI / 180.0; }

// 最大成分に基づいてベクトル方向を人間可読な文字列に変換
std::string vec_dir(const Eigen::Vector3d& v)
{
  int idx = 0;
  double mx = 0;
  for (int i = 0; i < 3; ++i) if (std::abs(v[i]) > mx) { mx = std::abs(v[i]); idx = i; }
  const char* names[3][2] = {{"+X (前方)","−X (後方)"},{"+Y (左方)","−Y (右方)"},{"+Z (上方)","−Z (下方)"}};
  return names[idx][v[idx] < 0 ? 1 : 0];
}

// RPY 角度から tcp_link 各軸の base_footprint 上の方向を説明
void explain_rpy(double roll_d, double pitch_d, double yaw_d)
{
  // tf2::setRPY の内部順: Rz(yaw)*Ry(pitch)*Rx(roll)
  Eigen::AngleAxisd Rz(d2r(yaw_d),   Eigen::Vector3d::UnitZ());
  Eigen::AngleAxisd Ry(d2r(pitch_d), Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd Rx(d2r(roll_d),  Eigen::Vector3d::UnitX());
  Eigen::Matrix3d R = (Rz * Ry * Rx).toRotationMatrix();

  // R の各列 = tcp_link の x/y/z 軸が base_footprint フレームで向く方向
  RCLCPP_INFO(LOG,
    "  RPY=(%.0f°, %.0f°, %.0f°) のとき tcp_link 軸の向き (base_footprint 基準):",
    roll_d, pitch_d, yaw_d);
  RCLCPP_INFO(LOG, "    tcp_link X → %s  (%.2f, %.2f, %.2f)",
    vec_dir(R.col(0)).c_str(), R(0,0), R(1,0), R(2,0));
  RCLCPP_INFO(LOG, "    tcp_link Y → %s  (%.2f, %.2f, %.2f)",
    vec_dir(R.col(1)).c_str(), R(0,1), R(1,1), R(2,1));
  RCLCPP_INFO(LOG, "    tcp_link Z → %s  (%.2f, %.2f, %.2f)  ← アプローチ方向",
    vec_dir(R.col(2)).c_str(), R(0,2), R(1,2), R(2,2));
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions opts;
  opts.automatically_declare_parameters_from_overrides(true);
  opts.parameter_overrides({{"use_sim_time", true}});
  auto node = rclcpp::Node::make_shared("survey_ik", opts);

  // automatically_declare_parameters_from_overrides(true) がlaunch引数を既に宣言済みのため
  // declare_parameter を再呼び出しすると ParameterAlreadyDeclaredException が発生する。
  // has_parameter でガードしてから get_parameter で値を取得する。
  if (node->has_parameter("x")) TARGET_X = node->get_parameter("x").as_double();
  if (node->has_parameter("y")) TARGET_Y = node->get_parameter("y").as_double();
  if (node->has_parameter("z")) TARGET_Z = node->get_parameter("z").as_double();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  moveit::planning_interface::MoveGroupInterface arm(node, "arm");
  arm.setPoseReferenceFrame("base_footprint");

  // =====================================================================
  // Part 1: RPY 軸の説明
  // =====================================================================
  RCLCPP_INFO(LOG, " ");
  RCLCPP_INFO(LOG, "╔══════════════════════════════════════════════╗");
  RCLCPP_INFO(LOG, "║  RPY 軸の意味 (base_footprint フレーム)      ║");
  RCLCPP_INFO(LOG, "╚══════════════════════════════════════════════╝");
  RCLCPP_INFO(LOG, "  base_footprint 座標系:");
  RCLCPP_INFO(LOG, "    +X = ロボット前方 (前進方向)");
  RCLCPP_INFO(LOG, "    +Y = ロボット左方");
  RCLCPP_INFO(LOG, "    +Z = 鉛直上方");
  RCLCPP_INFO(LOG, "  RPY 適用順: R = Rz(yaw) × Ry(pitch) × Rx(roll)");
  RCLCPP_INFO(LOG, "    Roll  (X 軸回転): ロボット前後軸周り");
  RCLCPP_INFO(LOG, "    Pitch (Y 軸回転): ロボット左右軸周り");
  RCLCPP_INFO(LOG, "    Yaw   (Z 軸回転): 鉛直軸周り (左右旋回)");
  RCLCPP_INFO(LOG, "  ※ mecanum3 の base_joint は Rz(-90°) なので");
  RCLCPP_INFO(LOG, "     アーム取り付け向きがロボット本体と 90° 異なる点に注意");
  RCLCPP_INFO(LOG, " ");

  // 指定姿勢の解析
  RCLCPP_INFO(LOG, "── 姿勢別の tcp_link 軸方向 ─────────────────────");
  explain_rpy(-90.0, 0.0,  90.0);   // ユーザー指定の現在値
  RCLCPP_INFO(LOG, " ");
  explain_rpy(-90.0, 0.0, -90.0);   // 過去に動作した値
  RCLCPP_INFO(LOG, " ");
  explain_rpy(  0.0, 0.0,   0.0);   // 参考: 無回転
  RCLCPP_INFO(LOG, " ");
  explain_rpy(  0.0, 90.0,  0.0);   // 参考: ピッチ 90°
  RCLCPP_INFO(LOG, " ");

  // =====================================================================
  // Part 2: FK サンプリングによる到達可能姿勢の調査
  // =====================================================================
  RCLCPP_INFO(LOG, "╔══════════════════════════════════════════════╗");
  RCLCPP_INFO(LOG, "║  FK サンプリング調査                          ║");
  RCLCPP_INFO(LOG, "╚══════════════════════════════════════════════╝");
  RCLCPP_INFO(LOG, "  目標位置: (%.3f, %.3f, %.3f) m", TARGET_X, TARGET_Y, TARGET_Z);
  RCLCPP_INFO(LOG, "  関節ステップ: %.0f°,  位置閾値: %.3f m", JOINT_STEP_DEG, POS_THRESH);
  RCLCPP_INFO(LOG, "  ※ 自己衝突チェックなし。目視確認を推奨。");
  RCLCPP_INFO(LOG, "  サンプリング中...");

  // ロボットモデルからフレッシュな状態を生成 (Gazebo の実状態に依存しない)
  // virtual_joint が fixed なので world = base_footprint; FK は base_footprint 基準になる
  auto robot_model = arm.getRobotModel();
  auto rs_base = std::make_shared<moveit::core::RobotState>(robot_model);
  rs_base->setToDefaultValues();
  rs_base->update();

  const auto* jmg = rs_base->getJointModelGroup("arm");
  const auto& jnames = jmg->getActiveJointModelNames();

  // ── 診断: デフォルト姿勢 (全関節=0/下限) での tcp_link 位置を表示 ──
  {
    const Eigen::Isometry3d& tcp = rs_base->getGlobalLinkTransform("tcp_link");
    RCLCPP_INFO(LOG, "  [診断] デフォルト姿勢の tcp_link 位置 (base_footprint 基準):");
    RCLCPP_INFO(LOG, "    (x=%.3f, y=%.3f, z=%.3f)",
      tcp.translation().x(), tcp.translation().y(), tcp.translation().z());
  }

  // 各関節の可動範囲を取得
  std::vector<double> jmin, jmax;
  RCLCPP_INFO(LOG, "  関節可動範囲:");
  for (const auto& n : jnames) {
    const auto& b = jmg->getJointModel(n)->getVariableBounds()[0];
    jmin.push_back(b.min_position_);
    jmax.push_back(b.max_position_);
    RCLCPP_INFO(LOG, "    %-10s  [%6.1f°, %6.1f°]",
      n.c_str(), r2d(b.min_position_), r2d(b.max_position_));
  }

  const double step = d2r(JOINT_STEP_DEG);
  const int N = static_cast<int>(jnames.size());
  Eigen::Vector3d tgt(TARGET_X, TARGET_Y, TARGET_Z);

  // サンプル数の事前計算
  long long total = 1;
  for (int i = 0; i < N; ++i) {
    long long cnt = static_cast<long long>((jmax[i] - jmin[i]) / step) + 1;
    total *= cnt;
  }
  RCLCPP_INFO(LOG, "  総サンプル数: %lld", total);

  // FK サンプリング
  auto rs = std::make_shared<moveit::core::RobotState>(*rs_base);

  // 重複排除用セット (RPY を RPY_ROUND_DEG 単位で丸めて管理)
  struct RpyKey {
    int r, p, y;
    bool operator<(const RpyKey& o) const {
      if (r != o.r) return r < o.r;
      if (p != o.p) return p < o.p;
      return y < o.y;
    }
  };
  struct RpyEntry {
    double roll, pitch, yaw;           // deg
    std::vector<double> joints;        // rad
    double dist;                       // 目標からの距離
  };
  std::map<RpyKey, RpyEntry> unique_rpy;
  long long sampled = 0;

  // 5重ループ (Joint_1..5)
  auto loop = [&](auto&& self, int depth, std::vector<double>& jv) -> void {
    if (depth == N) {
      ++sampled;
      rs->setJointGroupPositions(jmg, jv);
      rs->update();
      const Eigen::Isometry3d& tf = rs->getGlobalLinkTransform("tcp_link");
      Eigen::Vector3d pos = tf.translation();
      double dist = (pos - tgt).norm();
      if (dist < POS_THRESH) {
        // RPY を抽出 (tf2 getRPY と同じ規約)
        Eigen::Matrix3d R = tf.rotation();
        // 回転行列から ZYX オイラー角を取得
        // Eigen::eulerAngles(2,1,0) = ZYX
        Eigen::Vector3d ea = R.eulerAngles(2, 1, 0);  // [yaw, pitch, roll] in rad
        double roll_d  = r2d(ea[2]);
        double pitch_d = r2d(ea[1]);
        double yaw_d   = r2d(ea[0]);

        // 丸めてキーにする
        int kr = static_cast<int>(std::round(roll_d  / RPY_ROUND_DEG));
        int kp = static_cast<int>(std::round(pitch_d / RPY_ROUND_DEG));
        int ky = static_cast<int>(std::round(yaw_d   / RPY_ROUND_DEG));
        RpyKey key{kr, kp, ky};

        auto it = unique_rpy.find(key);
        if (it == unique_rpy.end() || dist < it->second.dist) {
          unique_rpy[key] = {roll_d, pitch_d, yaw_d, jv, dist};
        }
      }
      return;
    }
    for (double v = jmin[depth]; v <= jmax[depth] + 1e-9; v += step) {
      jv[depth] = v;
      self(self, depth + 1, jv);
    }
  };

  std::vector<double> jv(N, 0.0);
  loop(loop, 0, jv);

  RCLCPP_INFO(LOG, "  完了: サンプル数=%lld, 到達可能姿勢(重複排除後)=%zu",
    sampled, unique_rpy.size());

  if (unique_rpy.empty()) {
    RCLCPP_WARN(LOG, "  → 到達可能な姿勢が見つかりませんでした。");
    RCLCPP_WARN(LOG, "    POS_THRESH を大きくするか、ターゲット位置を変更してください。");
    rclcpp::shutdown(); spinner.join(); return 0;
  }

  // =====================================================================
  // Part 3: 結果の出力
  // =====================================================================
  RCLCPP_INFO(LOG, " ");
  RCLCPP_INFO(LOG, "╔══════════════════════════════════════════════╗");
  RCLCPP_INFO(LOG, "║  到達可能な姿勢一覧                           ║");
  RCLCPP_INFO(LOG, "╚══════════════════════════════════════════════╝");
  RCLCPP_INFO(LOG, "  (位置誤差 < %.3f m の配位, RPY を %.0f° 単位で重複排除)",
    POS_THRESH, RPY_ROUND_DEG);
  RCLCPP_INFO(LOG, " ");
  RCLCPP_INFO(LOG, "  %-30s | %-6s | tcp_link Z (アプローチ方向)",
    "moveTo の姿勢引数", "位置誤差");
  RCLCPP_INFO(LOG, "  %s", std::string(72, '-').c_str());

  // Roll/Pitch/Yaw の範囲集計
  double roll_min=1e9, roll_max=-1e9, pitch_min=1e9, pitch_max=-1e9, yaw_min=1e9, yaw_max=-1e9;

  for (const auto& [key, e] : unique_rpy) {
    // tcp_link Z 軸方向を計算
    Eigen::AngleAxisd Rz(d2r(e.yaw),   Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd Ry(d2r(e.pitch), Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd Rx(d2r(e.roll),  Eigen::Vector3d::UnitX());
    Eigen::Matrix3d R = (Rz * Ry * Rx).toRotationMatrix();
    Eigen::Vector3d tcp_z = R.col(2);

    RCLCPP_INFO(LOG,
      "  RPY=(%6.0f°, %5.0f°, %6.0f°)  | %.3f m | → %s",
      e.roll, e.pitch, e.yaw, e.dist, vec_dir(tcp_z).c_str());

    roll_min  = std::min(roll_min,  e.roll);
    roll_max  = std::max(roll_max,  e.roll);
    pitch_min = std::min(pitch_min, e.pitch);
    pitch_max = std::max(pitch_max, e.pitch);
    yaw_min   = std::min(yaw_min,   e.yaw);
    yaw_max   = std::max(yaw_max,   e.yaw);
  }

  RCLCPP_INFO(LOG, " ");
  RCLCPP_INFO(LOG, "  Roll  範囲: [%.0f°, %.0f°]", roll_min, roll_max);
  RCLCPP_INFO(LOG, "  Pitch 範囲: [%.0f°, %.0f°]", pitch_min, pitch_max);
  RCLCPP_INFO(LOG, "  Yaw   範囲: [%.0f°, %.0f°]", yaw_min, yaw_max);
  RCLCPP_INFO(LOG, " ");

  // ピック動作に適した姿勢の抽出 (tcp_link Z が前方 or 下方を向いているもの)
  RCLCPP_INFO(LOG, "  ── ピックに適した姿勢の候補 (tcp_Z が前方 or 下方) ──");
  bool found = false;
  for (const auto& [key, e] : unique_rpy) {
    Eigen::AngleAxisd Rz(d2r(e.yaw),   Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd Ry(d2r(e.pitch), Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd Rx(d2r(e.roll),  Eigen::Vector3d::UnitX());
    Eigen::Matrix3d R = (Rz * Ry * Rx).toRotationMatrix();
    Eigen::Vector3d tcp_z = R.col(2);
    // 前方 (+X) または下方 (-Z) に向いているものを抽出
    bool forward = tcp_z.x() > 0.5;
    bool downward = tcp_z.z() < -0.5;
    if (forward || downward) {
      RCLCPP_INFO(LOG,
        "  → moveTo(arm, %.2f, %.2f, %.2f, %.0f, %.0f, %.0f, speed);",
        TARGET_X, TARGET_Y, TARGET_Z, e.roll, e.pitch, e.yaw);
      found = true;
    }
  }
  if (!found) {
    RCLCPP_WARN(LOG, "  → 前方・下方向きの姿勢は見つかりませんでした。");
    RCLCPP_WARN(LOG, "    上記の到達可能姿勢一覧から手動で選択してください。");
  }

  RCLCPP_INFO(LOG, " ");
  RCLCPP_INFO(LOG, "=== 調査完了 ===");

  rclcpp::shutdown();
  spinner.join();
  return 0;
}
