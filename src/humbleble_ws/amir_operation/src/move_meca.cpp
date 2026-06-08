#include "rclcpp/rclcpp.hpp" // ROS 2の基本的なNode機能を提供
#include "std_msgs/msg/empty.hpp" // 空のメッセージ型
#include "behavior_tree_msgs/action/move_meca.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp" 
// #include "geometry_msgs/msg/Twist.hpp"  間違ってる書き方
#include "geometry_msgs/msg/twist.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include "rclcpp_action/rclcpp_action.hpp" // アクションサーバーを作成するためのライブラリ
#include <math.h>
#include <stdio.h>

// move_mecaクラスを定義
class MoveMeca : public rclcpp::Node
{
public:
    // エイリアス定義でコードの可読性を向上
    using MoveMecaAction = behavior_tree_msgs::action::MoveMeca;
    using GoalHandlemove_meca = rclcpp_action::ServerGoalHandle<MoveMecaAction>;

    // コンストラクタ: ノードの初期化とパブリッシャー、サブスクライバー、アクションサーバーの設定
    MoveMeca()
        : Node("MoveMeca") // ノード名を指定
    {
        // QoS 設定: BEST_EFFORT（データ損失を許容）
        rclcpp::QoS qos_profile(10); // キューサイズ10
        qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
        // パブリッシャーを初期化: "/amir/move_meca"トピックにコマンドを送信
        move_meca_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/rover_twist", 10);
        // サブスクライバーを初期化: 目標手先位置、オブジェクトの座標？
        move_meca_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", qos_profile, std::bind(&MoveMeca::odom_callback, this, std::placeholders::_1));
        // アクションサーバーを初期化
        action_server_ = rclcpp_action::create_server<MoveMecaAction>(
            this, // 現在のノードを指定
            "amir/move_meca", // アクション名
            std::bind(&MoveMeca::handle_goal, this, std::placeholders::_1, std::placeholders::_2), // ゴール受付コールバック
            std::bind(&MoveMeca::handle_cancel, this, std::placeholders::_1), // キャンセル受付コールバック
            std::bind(&MoveMeca::handle_accepted, this, std::placeholders::_1)); // ゴール処理コールバック

        // アクションサーバーの起動ログを出力
        RCLCPP_INFO(this->get_logger(), "****** MoveMeca action server started *****");
    }

private:
    // パブリッシャー: move_mecaコマンドを送信
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr move_meca_pub_;
    // サブスクライバー: 目標手先位置、オブジェクトの座標？を取得
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr move_meca_sub_;
    // アクションサーバー
    rclcpp_action::Server<MoveMecaAction>::SharedPtr action_server_;
    float meca_x;
    float meca_y;
    // ゴールを受け付けた際の処理
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &uuid, // ゴールのUUID
        std::shared_ptr<const MoveMecaAction::Goal> goal) // ゴールの内容
    {
        RCLCPP_INFO(this->get_logger(), "Received move_meca goal request");
        (void)uuid; // この場合、UUIDは使用しない
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; // ゴールを受け付け、実行を開始
    }
    // ゴールのキャンセルリクエストを受け付けた際の処理
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandlemove_meca> goal_handle) // ゴールハンドル
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle; // この場合、ゴールハンドルは使用しない
        return rclcpp_action::CancelResponse::ACCEPT; // キャンセルを受け付け
    }
    // ゴールを受け入れた後の処理
    void handle_accepted(const std::shared_ptr<GoalHandlemove_meca> goal_handle)
    {
        // 別スレッドでアクションを実行（非同期処理）
        std::thread([this, goal_handle]() {
            auto goal = goal_handle->get_goal();
            float G_x = goal->posi_x;
            float G_y = goal->posi_y;
    
            execute(goal_handle, G_x, G_y);
        }).detach();
    }
    void odom_callback(const std::shared_ptr<nav_msgs::msg::Odometry> msg)
    {
        meca_x = msg->pose.pose.position.x;
        meca_y = msg->pose.pose.position.y;
        // RCLCPP_INFO(this->get_logger(), "meca_x:%f", meca_x);
        // RCLCPP_INFO(this->get_logger(), "meca_y:%f", meca_y);
    }
    // アクションの実行処理
    void execute(const std::shared_ptr<GoalHandlemove_meca> goal_handle, float G_x, float G_y)
    {
        RCLCPP_INFO(this->get_logger(), "Executing move_meca action");
        auto result = std::make_shared<MoveMecaAction::Result>(); // 実行結果を格納する変数
        RCLCPP_INFO(this->get_logger(), "move_meca start");
        auto linear =  geometry_msgs::msg::Twist();
        rclcpp::Rate loop_rate(10); // 10Hz ループ
        //ここで計算
        // while (rclcpp::ok()) {
        //     float distance = std::sqrt((G_x - meca_x) * (G_x - meca_x) + (G_y - meca_y) * (G_y - meca_y));
        //     // RCLCPP_INFO(this->get_logger(), "distance:%f", distance);
        //     if (distance < 0.20) { // しきい値
        //         RCLCPP_INFO(this->get_logger(), "Reached target position");
        //         linear.linear.x = 0.0;
        //         linear.linear.y = 0.0;
        //         linear.linear.z = 0.0; 
        //         move_meca_pub_->publish(linear);
                
        //         RCLCPP_INFO(this->get_logger(), "move_meca ok");
        //         result->error_string = "success"; // 成功メッセージを設定
        //         result->success = true;
        //         goal_handle->succeed(result);

        //         linear.linear.x = 0.0;
        //         linear.linear.y = 0.0;
        //         linear.linear.z = 0.0; 
        //         move_meca_pub_->publish(linear);
        //         break;
        //     }
        //     //シンプルな比例制御 (P制御) で速度を計算
        //     linear.linear.x = 0.2 * (G_x - meca_x);
        //     linear.linear.y = 0.2 * (G_y - meca_y); 
        //     // x方向の速度を制限
        //     if (linear.linear.x > 0.2) {
        //         linear.linear.x = 0.2;
        //     } else if (linear.linear.x < -0.2) {
        //         linear.linear.x = -0.2;
        //     }
        //     // y方向の速度を制限
        //     if (linear.linear.y > 0.2) {
        //         linear.linear.y = 0.2;
        //     } else if (linear.linear.y < -0.2) {
        //         linear.linear.y = -0.2;
        //     }
        //     linear.linear.z = 0.0; // z方向は制御なし
        //     move_meca_pub_->publish(linear);
        //     loop_rate.sleep();
        // }
        while (rclcpp::ok()) {
            // 現在位置と目標位置の距離を計算
            float distance = std::sqrt((G_x - meca_x) * (G_x - meca_x) + (G_y - meca_y) * (G_y - meca_y));
            // RCLCPP_INFO(this->get_logger(), "distance: %f", distance);

            // 目標に十分近い場合
            if (distance < 0.20) { // しきい値（例: 20cm以内）
                RCLCPP_INFO(this->get_logger(), "Reached target position");
                // 複数回、ゼロ速度の指令を発行して完全停止を指示する
                for (int i = 0; i < 10; ++i) {
                    geometry_msgs::msg::Twist stop_msg;
                    stop_msg.linear.x = 0.0;
                    stop_msg.linear.y = 0.0;
                    stop_msg.linear.z = 0.0;
                    stop_msg.angular.x = 0.0;
                    stop_msg.angular.y = 0.0;
                    stop_msg.angular.z = 0.0;
                    move_meca_pub_->publish(stop_msg);
                    loop_rate.sleep();
                }
                result->error_string = "success";
                result->success = true;
                goal_handle->succeed(result);
                break;
            }

            // 目標までの距離が近づくと、ゲインを下げて急激な指令変化を抑制
            float gain = 0.2;
            if (distance < 0.5) {
                gain = 0.1;
            }

            // P制御により速度指令を計算
            linear.linear.x = gain * (G_x - meca_x);
            linear.linear.y = gain * (G_y - meca_y);
            linear.linear.z = 0.0;

            // angular成分も明示的にゼロに設定
            linear.angular.x = 0.0;
            linear.angular.y = 0.0;
            linear.angular.z = 0.0;

            // 速度の上限を設定（x方向）
            if (linear.linear.x > 0.2)
                linear.linear.x = 0.2;
            else if (linear.linear.x < -0.2)
                linear.linear.x = -0.2;
            // y方向も同様
            if (linear.linear.y > 0.2)
                linear.linear.y = 0.2;
            else if (linear.linear.y < -0.2)
                linear.linear.y = -0.2;

            move_meca_pub_->publish(linear);
            loop_rate.sleep();
        }
        RCLCPP_INFO(this->get_logger(), "move_meca finish");
    }

};

// メイン関数: ノードを起動
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv); // ROS 2の初期化
    auto node = std::make_shared<MoveMeca>(); // graspノードを作成
    rclcpp::spin(node); // ノードを実行
    rclcpp::shutdown(); // ROS 2の終了処理
    return 0;
}
