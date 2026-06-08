#include "rclcpp/rclcpp.hpp" // ROS 2の基本的なNode機能を提供
#include "std_msgs/msg/empty.hpp" // 空のメッセージ型（着陸コマンド用）
#include "amir_interfaces/msg/amir_cmd.hpp" // ドローンの位置情報（slam_pose用）
#include "behavior_tree_msgs/action/move_arm.hpp" // 着陸アクションのためのカスタムアクション型
#include "rclcpp_action/rclcpp_action.hpp" // アクションサーバーを作成するためのライブラリ
#include "amir_interfaces/msg/amir_sensor.hpp" //amirとのやり取りをするためのsub用型
#include <math.h>
#include <iostream>
#include <algorithm>
#include <unistd.h>
#include <stdio.h>
#include <chrono>
#include <thread>

// Landクラスを定義
class MoveArm : public rclcpp::Node
{
public:
    // エイリアス定義でコードの可読性を向上
    using MoveArmAction = behavior_tree_msgs::action::MoveArm;
    using GoalHandleLand = rclcpp_action::ServerGoalHandle<MoveArmAction>;

    // コンストラクタ: ノードの初期化とパブリッシャー、サブスクライバー、アクションサーバーの設定
    MoveArm()
        : Node("MoveArm_action_server") // ノード名を指定
    {
        // QoS 設定: BEST_EFFORT（データ損失を許容）
        rclcpp::QoS qos_profile(10); // キューサイズ10
        qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

        //パブリッシャの設定
        inversekinematics_publisher_ = this->create_publisher<amir_interfaces::msg::AmirCmd>("/motor_sub", 10);

        move_arm_sub_ = this->create_subscription<amir_interfaces::msg::AmirSensor>("/encoder_pub", qos_profile, std::bind(&MoveArm::angle_callback, this, std::placeholders::_1));

        // アクションサーバーを初期化
        action_server_ = rclcpp_action::create_server<MoveArmAction>(
            this, // 現在のノードを指定
            "amir/bt_move_arm", // アクション名
            std::bind(&MoveArm::handle_goal, this, std::placeholders::_1, std::placeholders::_2), // ゴール受付コールバック
            std::bind(&MoveArm::handle_cancel, this, std::placeholders::_1), // キャンセル受付コールバック
            std::bind(&MoveArm::handle_accepted, this, std::placeholders::_1)); // ゴール処理コールバック

        // アクションサーバーの起動ログを出力
        RCLCPP_INFO(this->get_logger(), "****** MoveArm action server started *****");
    }

private:
    // サブスクライバー
    rclcpp::Publisher<amir_interfaces::msg::AmirCmd>::SharedPtr inversekinematics_publisher_;
    // アクションサーバー
    rclcpp_action::Server<MoveArmAction>::SharedPtr action_server_;
    // サブスクライバー: 目標手先位置、オブジェクトの座標？を取得
    rclcpp::Subscription<amir_interfaces::msg::AmirSensor>::SharedPtr move_arm_sub_;
    double current_angle1;
    double current_angle2;
    double current_angle3;
    double current_angle4;
    double current_angle5;
    double current_angle6;


    //現在のangle[0]~angle[5]を取得
    void angle_callback(const amir_interfaces::msg::AmirSensor::SharedPtr msg)
    {
        // RCLCPP_INFO(this->get_logger(), "current_angl6");
        //取得した各関節角をcurrent_angle1~5に入れる
        current_angle1 = msg->angle[0];
        current_angle2 = msg->angle[1];
        current_angle3 = msg->angle[2];
        current_angle4 = msg->angle[3];
        current_angle5 = msg->angle[4];
        current_angle6 = msg->angle[5];
        // RCLCPP_INFO(this->get_logger(), "current_angl6:%f", current_angl6);
    }


    // ゴールを受け付けた際の処理
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &uuid, // ゴールのUUID
        std::shared_ptr<const MoveArmAction::Goal> goal) // ゴールの内容
    {
        // RCLCPP_INFO(this->get_logger(), "Received MoveArm goal request");
        (void)uuid;

        // ゴール内容をログに出力
        // RCLCPP_INFO(this->get_logger(), "Target Position -> x: %f, y: %f, z: %f", goal->x, goal->y, goal->z);

        // ゴールを受け付けて実行開始
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    // ゴールのキャンセルリクエストを受け付けた際の処理
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleLand> goal_handle) // ゴールハンドル
    {
        // RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle; // この場合、ゴールハンドルは使用しない
        return rclcpp_action::CancelResponse::ACCEPT; // キャンセルを受け付け
    }

    // ゴールを受け入れた後の処理
    void handle_accepted(const std::shared_ptr<GoalHandleLand> goal_handle)
    {
        std::thread([this, goal_handle]() {
            // ゴール値を取得
            auto goal = goal_handle->get_goal();
            float P_x = goal->x;
            float P_y = goal->y;
            float P_z = goal->z;

            // 実行
            execute(goal_handle, P_x, P_y, P_z);
        }).detach();
    }


    // アクションの実行処理
    void execute(const std::shared_ptr<GoalHandleLand> goal_handle, float P_x, float P_y, float P_z)
    {
        RCLCPP_INFO(this->get_logger(), "Executing Movearm action");

        auto result = std::make_shared<MoveArmAction::Result>(); // 実行結果を格納する変数

        // 逆運動学計算
        // RCLCPP_INFO(this->get_logger(), "Start inverse kinematics with x: %f, y: %f, z: %f", P_x, P_y, P_z);

        //amir740のリンクの長さと各関節角の入力と各変数の定義
        float L0 = 0.036;
        float L1 = 0.092;
        float L2 = 0.31;
        float L3 = 0.31;
        float L4 = 0.038;
        float L5 = 0.088;
        float D = 0.321;

        // 最大の関節角度
        float deg_max_1 = -340; // 0 to ~
        float deg_max_2 = -135;
        float deg_max_3 = 160;
        float deg_max_4 = -195;
        float deg_max_5 = 316;

        float theta234;
        float theta234_;

        float a;
        float b;
        float c;
        float d;

        float x;
        float z;
        
        //単位行列とする
        float r_11 = 1.0;
        float r_12 = 0.0;
        float r_13 = 0.0;
        float r_21 = 0.0;
        float r_22 = 1.0;
        float r_23 = 0.0;
        float r_31 = 0.0;
        float r_32 = 0.0;
        float r_33 = 1.0;

        // 手首を-45度に固定
        // float r_11 = 1/sqrt(2);
        // float r_12 = 0.0;
        // float r_13 = -1/sqrt(2);
        // float r_21 = 0.0;
        // float r_22 = 1.0;
        // float r_23 = 0.0;
        // float r_31 = 1/sqrt(2);
        // float r_32 = 0.0;
        // float r_33 = 1/sqrt(2);

        // rad出力
        float theta1;
        float theta2;
        float theta3;
        float theta4;
        float theta5;
        
        // deg出力
        float theta1_deg;
        float theta2_deg;
        float theta3_deg;
        float theta4_deg;
        float theta5_deg;

        // mrad出力
        float theta1_mrad;
        float theta2_mrad;
        float theta3_mrad;
        float theta4_mrad;
        float theta5_mrad;

        // 目標位置にて入力した値と順運動学より出力した値の誤差計算
        float error_x;
        float error_y;
        float error_z;
        float error_r_11;
        float error_r_22;
        float error_r_33;

        //逆運動学の計算に必要な変数の計算
        theta1 = (atan2(P_y, P_x));
        theta234 = atan((cos(theta1) * r_13+sin(theta1) * r_23) / r_33);
        theta234_ = - theta234 - (M_PI) / 2;
        x = (P_x / cos(theta1)) - L0;
        z = P_z - (D + L1);
        a = z - (L4 + L5) * sin(theta234_);
        b = x - (L4 + L5) * cos(theta234_);
        c = ((b * b) + (a * a) + (L2 * L2) - (L3 * L3)) / (2 * L2);
        d = ((b * b) + (a * a) + (L3 * L3) - (L2 * L2)) / (2 * L2);

        //逆運動学でamirの各関節角度を計算する
        // 各thetaの範囲内であれば各関節角度を計算する
        theta2 = (acos(c / (sqrt(a * a + b * b))) + acos(b / (sqrt(a * a + b * b))));
        theta3 = (-M_PI + 2 * (asin(c / (sqrt((a * a) + (b * b))))));
        theta4 = ((atan2((cos(theta1) * r_13 + sin(theta1) * r_23),r_33) - theta2 - theta3));
        theta5 = (atan2((cos(theta1) * r_21 - sin(theta1) * r_11),(cos(theta1) * r_22 - sin(theta1) * r_12)));

        // 順運動学にて出力する位置座標と姿勢行列の値
        float pos_x;
        float pos_y;
        float pos_z;
        float r_11_;
        float r_22_;
        float r_33_;

        float vel0 =400;


        // degの計算
        theta1_deg = ((theta1 * 180) / (M_PI));
        theta2_deg = ((theta2 * 180) / (M_PI));
        theta3_deg = ((theta3 * 180) / (M_PI));
        theta4_deg = ((theta4 * 180) / (M_PI));
        theta5_deg = ((theta5 * 180) / (M_PI));

        // RCLCPP_INFO(this->get_logger(), "theta1_deg:%f, theta2_deg:%f, theta3_deg:%f, theta4_deg:%f, theta5_deg:%f", theta1_deg, theta2_deg, theta3_deg, theta4_deg, theta5_deg);

        // 最大関節角度を考慮
        float off_theta1_deg_result = std::clamp(theta1_deg, deg_max_1 / 2, -deg_max_1 / 2); // 負から0
        float off_theta2_deg_result = std::clamp(theta2_deg, 0.0f, -deg_max_2); // 負から0
        float off_theta3_deg_result = std::clamp(theta3_deg, -deg_max_3, 0.0f); // 0から正
        float off_theta4_deg_result = std::clamp(theta4_deg, -30.0f, 165.0f); // 負から0
        float off_theta5_deg_result = std::clamp(theta5_deg, -deg_max_5 / 2, deg_max_5 / 2); // 0から正
        // RCLCPP_INFO(this->get_logger(), "1:%f, 2:%f, 3:%f, 4:%f, 5:%f", off_theta1_deg_result, off_theta2_deg_result, off_theta3_deg_result, off_theta4_deg_result, off_theta5_deg_result);

        // radに変更
        float theta1_rad = (off_theta1_deg_result * M_PI) / 180;
        float theta2_rad = (off_theta2_deg_result * M_PI) / 180;
        float theta3_rad = (off_theta3_deg_result * M_PI) / 180;
        float theta4_rad = (off_theta4_deg_result * M_PI) / 180;
        float theta5_rad = (off_theta5_deg_result * M_PI) / 180;

        // 順運動学の計算
        pos_x = cos(theta1_rad) * (L3 * cos(theta2_rad + theta3_rad) + L2 * cos(theta2_rad) + L0 + (L4 + L5) * sin(theta2_rad + theta3_rad + theta4_rad));
        pos_y = sin(theta1_rad) * (L3 * cos(theta2_rad + theta3_rad) + L2 * cos(theta2_rad) + L0 + (L4 + L5) * sin(theta2_rad + theta3_rad + theta4_rad));
        pos_z = L3 * sin(theta2_rad + theta3_rad) + L2 * sin(theta2_rad) + (D + L1) - (L4 + L5) * cos(theta2_rad + theta3_rad + theta4_rad);

        r_11_ = cos(theta1_rad) * cos(theta2_rad + theta3_rad + theta4_rad) * cos(theta5_rad) + sin(theta1_rad) * sin(theta5_rad);
        r_22_ = -sin(theta1_rad) * cos(theta2_rad + theta3_rad + theta4_rad) * sin(theta5_rad) + cos(theta1_rad) * cos(theta5_rad);
        r_33_ = cos(theta2_rad + theta3_rad + theta4_rad);

        // RCLCPP_INFO(this->get_logger(), "pos_x:%f, pos_y:%f, pos_z:%f, r_11_:%f, r_22_:%f, r_33_:%f", pos_x, pos_y, pos_z, r_11_, r_22_, r_33_);

        // エンドエフェクタの誤差確認
        error_x = P_x - pos_x;
        error_y = P_y - pos_y;
        error_z = P_z - pos_z;
        // エンドエフェクタの単位行列の誤差
        error_r_11 = r_11 - r_11_;
        error_r_22 = r_22 - r_22_;
        error_r_33 = r_33 - r_33_;

        // 実機のオフセットに合わせる
        float off_theta1_deg = (deg_max_1 / 2) + off_theta1_deg_result;
        float off_theta2_deg = (-135) + off_theta2_deg_result;
        float off_theta3_deg = (160) + off_theta3_deg_result;
        float off_theta4_deg = (-75) + off_theta4_deg_result;
        float off_theta5_deg = (deg_max_5 / 2) + off_theta5_deg_result;
        // RCLCPP_INFO(this->get_logger(), "off_theta1_deg:%f, off_theta2_deg:%f, off_theta3_deg:%f, off_theta4_deg:%f, off_theta5_deg:%f", off_theta1_deg, off_theta2_deg, off_theta3_deg, off_theta4_deg, off_theta5_deg);

        // mradに変更
        // theta1_mrad = (off_theta1_deg_result * 1000 * M_PI) / 180;
        // theta2_mrad = (off_theta2_deg_result * 1000 * M_PI) / 180;
        // theta3_mrad = (off_theta3_deg_result * 1000 * M_PI) / 180;
        // theta4_mrad = (off_theta4_deg_result * 1000 * M_PI) / 180;
        // theta5_mrad = (off_theta4_deg_result * 1000 * M_PI) / 180;

        theta1_mrad = (off_theta1_deg * 1000 * M_PI) / 180;
        theta2_mrad = (off_theta2_deg * 1000 * M_PI) / 180;
        theta3_mrad = (off_theta3_deg * 1000 * M_PI) / 180;
        theta4_mrad = (off_theta4_deg * 1000 * M_PI) / 180;
        theta5_mrad = ((deg_max_5 / 2 )* 1000 * M_PI) / 180;

        // もしエンドエフェクタの位置誤差が小さければ成功
        float diff = sqrt((error_x) * (error_x) + (error_y) * (error_y) + (error_z) * (error_z));
        RCLCPP_INFO(this->get_logger(), "diff:%f", diff);
        if (diff <= 0.03)
        {
            // 結果を送信する
            auto goal = amir_interfaces::msg::AmirCmd();

            //順番関係なく、動かしたいﾄｷ
            // goal.angle[0] = theta1_mrad;
            // goal.angle[1] = theta2_mrad;
            // goal.angle[2] = theta3_mrad;
            // goal.angle[3] = theta4_mrad;
            // goal.angle[4] = theta5_mrad;    //現状0°に固定
            // goal.angle[5] = current_angle6;
            // goal.vel[0] = 200.0;
            // goal.vel[1] = 200.0;
            // goal.vel[2] = 200.0;
            // goal.vel[3] = 200.0;
            // goal.vel[4] = 400.0;
            // goal.vel[5] = 200.0;
            // inversekinematics_publisher_->publish(goal);

            //toru toki
            if(current_angle2 > -800){
                goal.angle[0] = theta1_mrad;
                goal.angle[1] = current_angle2;
                goal.angle[2] = current_angle3;
                goal.angle[3] = current_angle4;
                goal.angle[4] = current_angle5;    //現状0°に固定
                goal.angle[5] = current_angle6;
                goal.vel[0] = vel0;
                goal.vel[1] = 200.0;
                goal.vel[2] = 200.0;
                goal.vel[3] = 200.0;
                goal.vel[4] = 400.0;
                goal.vel[5] = 200.0;
                for (int i = 0; i < 6; ++i) {
                inversekinematics_publisher_->publish(goal);
                }
                while (rclcpp::ok()){
                    float error_angle_1 = theta1_mrad - current_angle1;
                    // RCLCPP_INFO(this->get_logger(), "error_angle_1:%f", error_angle_1);
                    if (fabs(error_angle_1)< 20)
                    {                       
                        break;
                    }
                }
            
                goal.angle[0] = current_angle1;
                goal.angle[1] = current_angle2;
                goal.angle[2] = theta3_mrad;
                goal.angle[3] = current_angle4;
                goal.angle[4] = current_angle5;    //現状0°に固定
                goal.angle[5] = current_angle6;
                goal.vel[0] = vel0;
                goal.vel[1] = 200.0;
                goal.vel[2] = 200.0;
                goal.vel[3] = 200.0;
                goal.vel[4] = 400.0;
                goal.vel[5] = 200.0;
                for (int o = 0; o < 6; ++o) {
                inversekinematics_publisher_->publish(goal);
                }

                while (rclcpp::ok()){
                    float error_angle_3 = theta3_mrad - current_angle3;
                    RCLCPP_INFO(this->get_logger(), "error_angle_3:%f", error_angle_3);
                    if (fabs(error_angle_3)< 1500)
                    {                       
                        break;
                    }
                }

                std::this_thread::sleep_for(std::chrono::seconds(1));

                // float error_angle_1 = current_angle1 - theta1_mrad;
                // RCLCPP_INFO(this->get_logger(), "error_angle_1:%f", error_angle_1);
                goal.angle[0] = current_angle1;
                goal.angle[1] = theta2_mrad;
                goal.angle[2] = theta3_mrad;
                goal.angle[3] = theta4_mrad;
                goal.angle[4] = theta5_mrad;    //現状0°に固定
                goal.angle[5] = current_angle6;
                goal.vel[0] = vel0;
                goal.vel[1] = 200.0;
                goal.vel[2] = 200.0;
                goal.vel[3] = 200.0;
                goal.vel[4] = 400.0;
                goal.vel[5] = 200.0;
                for (int p = 0; p < 6; ++p) {
                inversekinematics_publisher_->publish(goal);
                }
            }
            else //modoru toki
            {
                goal.angle[0] = current_angle1;
                goal.angle[1] = theta2_mrad;
                goal.angle[2] = theta3_mrad;
                goal.angle[3] = theta4_mrad;
                goal.angle[4] = theta5_mrad;    //現状0°に固定
                goal.angle[5] = current_angle6;
                goal.vel[0] = vel0;
                goal.vel[1] = 200.0;
                goal.vel[2] = 200.0;
                goal.vel[3] = 200.0;
                goal.vel[4] = 400.0;
                goal.vel[5] = 200.0;
                for (int l = 0; l < 6; ++l) {
                inversekinematics_publisher_->publish(goal);
                }

                while (rclcpp::ok()){
                    float error_angle_2 = theta2_mrad - current_angle2;
                    if (fabs(error_angle_2)< 100)
                    {                       
                        break;
                    }
                }

                goal.angle[0] = theta1_mrad;
                goal.angle[1] = current_angle2;
                goal.angle[2] = current_angle3;
                goal.angle[3] = current_angle4;
                goal.angle[4] = current_angle5;    //現状0°に固定
                goal.angle[5] = current_angle6;
                goal.vel[0] = vel0;
                goal.vel[1] = 200.0;
                goal.vel[2] = 200.0;
                goal.vel[3] = 200.0;
                goal.vel[4] = 400.0;
                goal.vel[5] = 200.0;
                for (int k = 0; k < 6; ++k) {
                inversekinematics_publisher_->publish(goal);
                }

            }


            sleep(10);
            result->success = true;
            result->error_string = "success"; // 成功メッセージを設定
            goal_handle->succeed(result); // ゴール成功を通知
            RCLCPP_INFO(this->get_logger(), "Action succeeded");

            // while (rclcpp::ok()){

            //     // 現在の関節角度と目標の関節角度の誤差
            //     float error_angle_1 = current_angle1 - theta1_mrad;
            //     float error_angle_2 = current_angle2 - theta2_mrad;
            //     float error_angle_3 = current_angle3 - theta3_mrad;
            //     float error_angle_4 = current_angle4 - theta4_mrad;
            //     float error_angle_5 = current_angle5 - theta5_mrad;

            //     // もし関節角度の誤差が小さければ成功
            //     float angle_diff = sqrt((error_angle_1) * (error_angle_1) + (error_angle_2) * (error_angle_2) + (error_angle_3) * (error_angle_3) + (error_angle_4) * (error_angle_4) + (error_angle_5) * (error_angle_5));

            //     if (angle_diff <= 0.2)
            //     {
            //         inversekinematics_publisher_->publish(goal);
            //         result->success = true;
            //         result->error_string = "success"; // 成功メッセージを設定
            //         goal_handle->succeed(result); // ゴール成功を通知
            //         RCLCPP_INFO(this->get_logger(), "Action succeeded");
            //         break;
            //     }
            //     RCLCPP_INFO(this->get_logger(), "MoveArm running angle_diff:%f",angle_diff);
            // }
        }
        else
        {
            // 失敗したらどうしよう...(もう少し近づくとか？？)
            RCLCPP_INFO(this->get_logger(), "Action Ｆ");
        }
        
    }
};

// メイン関数: ノードを起動
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv); // ROS 2の初期化
    auto node = std::make_shared<MoveArm>(); // MoveArmノードを作成
    rclcpp::spin(node); // ノードを実行
    rclcpp::shutdown(); // ROS 2の終了処理
    return 0;
}
