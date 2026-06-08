#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "behavior_tree_msgs/action/before_arm.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include <math.h>
#include <iostream>
#include <algorithm>
#include <chrono>
#include <thread>

class BeforeArm : public rclcpp::Node
{
public:
    using BeforeArmAction = behavior_tree_msgs::action::BeforeArm;
    using GoalHandleLand = rclcpp_action::ServerGoalHandle<BeforeArmAction>;
    using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
    using TrajectoryGoalHandle = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

    BeforeArm()
        : Node("BeforeArm_action_server")
    {
        rclcpp::QoS qos_profile(10);
        qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

        // /joint_states を購読して現在の関節角度（rad）を取得
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", qos_profile, std::bind(&BeforeArm::joint_state_callback, this, std::placeholders::_1));

        // アーム制御用のアクションクライアント（コントローラへ送信）
        trajectory_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
            this, "/arm_controller/follow_joint_trajectory");

        // Behavior Tree からの指令を受け付けるアクションサーバー
        action_server_ = rclcpp_action::create_server<BeforeArmAction>(
            this,
            "amir/bt_before_arm",
            std::bind(&BeforeArm::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&BeforeArm::handle_cancel, this, std::placeholders::_1),
            std::bind(&BeforeArm::handle_accepted, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "****** BeforeArm action server started *****");
    }

private:
    rclcpp_action::Server<BeforeArmAction>::SharedPtr action_server_;
    rclcpp_action::Client<FollowJointTrajectory>::SharedPtr trajectory_client_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

    // 各関節の現在角度 (rad)
    double current_angle1 = 0.0;
    double current_angle2 = 0.0;
    double current_angle3 = 0.0;
    double current_angle4 = 0.0;
    double current_angle5 = 0.0;

    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        // joint_statesは順序が保証されない場合があるため、名前でマッチングして取得します
        for (size_t i = 0; i < msg->name.size(); ++i) {
            if (msg->name[i] == "Joint_1") current_angle1 = msg->position[i];
            else if (msg->name[i] == "Joint_2") current_angle2 = msg->position[i];
            else if (msg->name[i] == "Joint_3") current_angle3 = msg->position[i];
            else if (msg->name[i] == "Joint_4") current_angle4 = msg->position[i];
            else if (msg->name[i] == "Joint_5") current_angle5 = msg->position[i];
        }
    }

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const BeforeArmAction::Goal> goal)
    {
        (void)uuid;
        (void)goal;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleLand> goal_handle)
    {
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleLand> goal_handle)
    {
        std::thread([this, goal_handle]() {
            auto goal = goal_handle->get_goal();
            float P_x = goal->pose_obj.pose.position.x;
            float P_y = goal->pose_obj.pose.position.y;
            float P_z = goal->pose_obj.pose.position.z;
            execute(goal_handle, P_x, P_y, P_z);
        }).detach();
    }

    void execute(const std::shared_ptr<GoalHandleLand> goal_handle, float P_x, float P_y, float P_z)
    {
        RCLCPP_INFO(this->get_logger(), "Executing BeforeArm action");
        auto result = std::make_shared<BeforeArmAction::Result>();

        // ----- 逆運動学計算 (以前のロジックをそのまま使用) -----
        float L0 = 0.036, L1 = 0.092, L2 = 0.31, L3 = 0.31, L4 = 0.038, L5 = 0.088, D = 0.321;
        float deg_max_1 = -340, deg_max_2 = -135, deg_max_3 = 160, deg_max_4 = -195, deg_max_5 = 316;
        float theta234, theta234_, a, b, c, d, x, z;
        
        float r_11 = 1.0, r_12 = 0.0, r_13 = 0.0;
        float r_21 = 0.0, r_22 = 1.0, r_23 = 0.0;
        float r_31 = 0.0, r_32 = 0.0, r_33 = 1.0;

        float theta1, theta2, theta3, theta4, theta5;
        float theta1_deg, theta2_deg, theta3_deg, theta4_deg, theta5_deg;
        float error_x, error_y, error_z, error_r_11, error_r_22, error_r_33;

        theta1 = atan2(P_y, P_x);
        theta234 = atan((cos(theta1) * r_13 + sin(theta1) * r_23) / r_33);
        theta234_ = -theta234 - (M_PI) / 2;
        x = (P_x / cos(theta1)) - L0;
        z = P_z - (D + L1);
        a = z - (L4 + L5) * sin(theta234_);
        b = x - (L4 + L5) * cos(theta234_);
        c = ((b * b) + (a * a) + (L2 * L2) - (L3 * L3)) / (2 * L2);
        d = ((b * b) + (a * a) + (L3 * L3) - (L2 * L2)) / (2 * L2);

        theta2 = (acos(c / (sqrt(a * a + b * b))) + acos(b / (sqrt(a * a + b * b))));
        theta3 = (-M_PI + 2 * (asin(c / (sqrt((a * a) + (b * b))))));
        theta4 = ((atan2((cos(theta1) * r_13 + sin(theta1) * r_23), r_33) - theta2 - theta3));
        theta5 = (atan2((cos(theta1) * r_21 - sin(theta1) * r_11), (cos(theta1) * r_22 - sin(theta1) * r_12)));

        theta1_deg = ((theta1 * 180) / M_PI);
        theta2_deg = ((theta2 * 180) / M_PI);
        theta3_deg = ((theta3 * 180) / M_PI);
        theta4_deg = ((theta4 * 180) / M_PI);
        theta5_deg = ((theta5 * 180) / M_PI);

        float off_theta1_deg_result = std::clamp(theta1_deg, deg_max_1 / 2, -deg_max_1 / 2);
        float off_theta2_deg_result = std::clamp(theta2_deg, 0.0f, -deg_max_2);
        float off_theta3_deg_result = std::clamp(theta3_deg, -deg_max_3, 0.0f);
        float off_theta4_deg_result = std::clamp(theta4_deg, -30.0f, 165.0f);
        float off_theta5_deg_result = std::clamp(theta5_deg, -deg_max_5 / 2, deg_max_5 / 2);

        float theta1_rad = (off_theta1_deg_result * M_PI) / 180;
        float theta2_rad = (off_theta2_deg_result * M_PI) / 180;
        float theta3_rad = (off_theta3_deg_result * M_PI) / 180;
        float theta4_rad = (off_theta4_deg_result * M_PI) / 180;
        float theta5_rad = (off_theta5_deg_result * M_PI) / 180;

        float pos_x = cos(theta1_rad) * (L3 * cos(theta2_rad + theta3_rad) + L2 * cos(theta2_rad) + L0 + (L4 + L5) * sin(theta2_rad + theta3_rad + theta4_rad));
        float pos_y = sin(theta1_rad) * (L3 * cos(theta2_rad + theta3_rad) + L2 * cos(theta2_rad) + L0 + (L4 + L5) * sin(theta2_rad + theta3_rad + theta4_rad));
        float pos_z = L3 * sin(theta2_rad + theta3_rad) + L2 * sin(theta2_rad) + (D + L1) - (L4 + L5) * cos(theta2_rad + theta3_rad + theta4_rad);

        error_x = P_x - pos_x;
        error_y = P_y - pos_y;
        error_z = P_z - pos_z;

        float diff = sqrt((error_x) * (error_x) + (error_y) * (error_y) + (error_z) * (error_z));
        RCLCPP_INFO(this->get_logger(), "Kinematics error diff: %f", diff);

        if (diff <= 0.03)
        {
            // HWIFのオフセットを含めた最終的な目標角度 (rad) の算出
            float off_theta1_deg = (deg_max_1 / 2) + off_theta1_deg_result;
            float off_theta2_deg = (-135) + off_theta2_deg_result;
            float off_theta3_deg = (160) + off_theta3_deg_result;
            float off_theta4_deg = (-75) + off_theta4_deg_result;
            
            // コントローラに送信するための最終的なラジアン値
            double target_q1 = off_theta1_deg * M_PI / 180.0;
            double target_q2 = off_theta2_deg * M_PI / 180.0;
            double target_q3 = off_theta3_deg * M_PI / 180.0;
            double target_q4 = off_theta4_deg * M_PI / 180.0;
            double target_q5 = (deg_max_5 / 2) * M_PI / 180.0; // 現状0°固定に相当する元の仕様を踏襲

            // アクションサーバーの起動確認
            if (!trajectory_client_->wait_for_action_server(std::chrono::seconds(3))) {
                RCLCPP_ERROR(this->get_logger(), "Trajectory Action Server not available.");
                result->success = false;
                goal_handle->abort(result);
                return;
            }

            auto trajectory_goal = FollowJointTrajectory::Goal();
            trajectory_goal.trajectory.joint_names = {
                "Joint_1", "Joint_2", "Joint_3", "Joint_4", "Joint_5"
            };

            // ----- 軌道（Trajectory）の作成 -----
            // 以前の while ループによる順番待ちを、時間指定の「ポイント」として定義します。
            
            // ポイント1 (2秒後): Joint 1 のみ目標位置へ、他は現在位置を維持
            trajectory_msgs::msg::JointTrajectoryPoint point1;
            point1.positions = {target_q1, current_angle2, current_angle3, current_angle4, current_angle5};
            point1.time_from_start.sec = 2;
            trajectory_goal.trajectory.points.push_back(point1);

            // ポイント2 (4秒後): Joint 3 を目標位置へ (Joint 1 は目標位置を維持)
            trajectory_msgs::msg::JointTrajectoryPoint point2;
            point2.positions = {target_q1, current_angle2, target_q3, current_angle4, current_angle5};
            point2.time_from_start.sec = 4;
            trajectory_goal.trajectory.points.push_back(point2);

            // ポイント3 (7秒後): 全ての Joint を最終目標位置へ
            trajectory_msgs::msg::JointTrajectoryPoint point3;
            point3.positions = {target_q1, target_q2, target_q3, target_q4, target_q5};
            point3.time_from_start.sec = 7;
            trajectory_goal.trajectory.points.push_back(point3);

            // ----- コントローラへの送信と結果の待機 -----
            auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
            
            send_goal_options.result_callback = 
                [this, goal_handle, result](const TrajectoryGoalHandle::WrappedResult & traj_result) {
                    if (traj_result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                        RCLCPP_INFO(this->get_logger(), "Trajectory execution succeeded!");
                        result->success = true;
                        result->error_string = "success";
                        goal_handle->succeed(result);
                    } else {
                        RCLCPP_ERROR(this->get_logger(), "Trajectory execution failed.");
                        result->success = false;
                        result->error_string = "failed";
                        goal_handle->abort(result);
                    }
                };

            // 非同期で軌道を送信
            trajectory_client_->async_send_goal(trajectory_goal, send_goal_options);
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Action FAILED (Kinematics diff too large)");
            result->success = false;
            goal_handle->abort(result);
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BeforeArm>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}