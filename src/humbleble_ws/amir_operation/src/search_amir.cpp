#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "behavior_tree_msgs/action/search_amir.hpp"
#include "amir_interfaces/msg/amir_cmd.hpp"
#include "amir_interfaces/msg/amir_sensor.hpp" 
#include <chrono>
#include <thread>
#include <cmath>


using SearchAction = behavior_tree_msgs::action::SearchAmir;

class SearchServer : public rclcpp::Node
{
public:
    SearchServer() : Node("search_server")
    {
        // QoS 設定: BEST_EFFORT（データ損失を許容）
        rclcpp::QoS qos_profile(10); // キューサイズ10
        qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

        // Searchアクションサーバーを作成
        action_server_ = rclcpp_action::create_server<SearchAction>(
            this,
            "search",
            std::bind(&SearchServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&SearchServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&SearchServer::handle_accepted, this, std::placeholders::_1));

        // 関節角のパブリッシャー
        joint_pub_ = this->create_publisher<amir_interfaces::msg::AmirCmd>("/motor_sub", 10);
        // 関節角のサブスクライバー
        joint_sub_ = this->create_subscription<amir_interfaces::msg::AmirSensor>("/encoder_pub", qos_profile, std::bind(&SearchServer::angle_callback, this, std::placeholders::_1));
        // アクションサーバーの起動ログを出力
        RCLCPP_INFO(this->get_logger(), "****** Search action server started ******");

        //アームを開いた状態にする
        // gripper_open();
        
    }

private:
    rclcpp_action::Server<SearchAction>::SharedPtr action_server_;
    rclcpp::Publisher<amir_interfaces::msg::AmirCmd>::SharedPtr joint_pub_;
    rclcpp::Subscription<amir_interfaces::msg::AmirSensor>::SharedPtr joint_sub_;

    // std::vector<float> angle_;
    // std::vector<float> vel_;

    //encoder_pubから取得した各関節角
    double current_angle1;
    double current_angle2;
    double current_angle3;
    double current_angle4;
    double current_angle5;
    double current_angle6;

    double target_angle1;
    bool angle_received_;
    bool nyossu_ = false;

    

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const SearchAction::Goal> goal)
    {        
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<SearchAction>> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Cancel request received");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<SearchAction>> goal_handle)
    {
        std::thread([this, goal_handle]() {
            this->execute(goal_handle);
        }).detach();
    }

    // bool is_reached(float target, float actual, float threshold = 100.0f)
    // {
    //     return std::fabs(target - actual) < threshold;
    // }

    void angle_callback(const amir_interfaces::msg::AmirSensor::SharedPtr msg)
    {
        // RCLCPP_INFO(this->get_logger(), "current_angle6");
        //取得した各関節角をcurrent_angle1~5に入れる
        current_angle1 = msg->angle[0];
        current_angle2 = msg->angle[1];
        current_angle3 = msg->angle[2];
        current_angle4 = msg->angle[3];
        current_angle5 = msg->angle[4];
        current_angle6 = msg->angle[5];
        angle_received_ = true;
        // RCLCPP_INFO(this->get_logger(), "current_angle6:%f", current_angle1);
    }

    void publish_target_joint(double target_angle1)
    {
        auto goal = amir_interfaces::msg::AmirCmd();
        goal.angle[0] = target_angle1;
        goal.angle[1] = current_angle2;
        goal.angle[2] = current_angle3;
        goal.angle[3] = current_angle4;
        goal.angle[4] = current_angle5; // 固定
        goal.angle[5] = current_angle6;

        goal.vel[0] = 500.0;
        goal.vel[1] = 200.0;
        goal.vel[2] = 200.0;
        goal.vel[3] = 200.0;
        goal.vel[4] = 400.0;
        goal.vel[5] = 200.0;

        for (int i = 0; i < 10; ++i) {
            joint_pub_->publish(goal);
            rclcpp::sleep_for(std::chrono::milliseconds(10));  // 少し間隔を入れるのが安全
        }

        RCLCPP_INFO(this->get_logger(), "🔁 目標関節角を送信しました: angle[0] = %.1f", target_angle1);
    }


   
    void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<SearchAction>> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing search action");
        auto result = std::make_shared<SearchAction::Result>();
        rclcpp::Rate loop_rate(10);

        int direction = 1;  // 1: 増加, -1: 減少
        target_angle1 =  current_angle1;  // 初期値
        const double step = 300.0;  // ステップ量
        const double max_angle = -600.0;
        const double min_angle = -5434.0;

        // while (rclcpp::ok()) {
        //     if (goal_handle->is_canceling()) {
        //         result->success = false;
        //         goal_handle->canceled(result);
        //         RCLCPP_INFO(this->get_logger(), "Goal canceled");
        //         return;
        //     }
            

        //     publish_target_joint(target_angle1);
            

        //     // while (rclcpp::ok()){
        //     //     if (!angle_received_) {
        //     //         rclcpp::sleep_for(std::chrono::milliseconds(50));
        //     //         continue;
        //     //     }

        //     // }

        //     int wait_count = 0;
        //     while (rclcpp::ok() && wait_count < 100) {
        //         if (!angle_received_) {
        //             rclcpp::sleep_for(std::chrono::milliseconds(50));
        //             wait_count++;
        //             continue;
        //         }

        //     if (std::fabs(target_angle1 - current_angle1) < 50.0) {
        //         RCLCPP_INFO(this->get_logger(), "✅ 到達: angle[0] = %.1f", current_angle1);
        //         rclcpp::sleep_for(std::chrono::seconds(1));

    
        //         // 次のステップへ進める
        //         target_angle1 = current_angle1 + direction * step;
    
        //         // 範囲外なら方向を反転（往復する場合）
        //         if (target_angle1 > max_angle) {
        //             target_angle1 = max_angle;
        //             direction = -1;
        //         } else if (target_angle1 < min_angle) {
        //             target_angle1 = min_angle;
        //             direction = 1;
        //             nyossu_ = true;
        //         }
        //         break;
        //     }


        //     if (nyossu_ == true)
        //     {
        //         // sleep(2);
        //         rclcpp::sleep_for(std::chrono::seconds(2));
        //         result->success = true;
        //         result->error_string = "success"; // 成功メッセージを設定
        //         goal_handle->succeed(result); // ゴール成功を通知
        //         RCLCPP_INFO(this->get_logger(), "Action succeeded");
        //         return; 
        //     }
        //     loop_rate.sleep();

        // }
        // // result->success = false;
        // // goal_handle->abort(result);
        
            
        // }
        ///////////////////////////////////////////////rinji
        auto joint = amir_interfaces::msg::AmirCmd();
        joint.angle[0] = -2967.0;
        joint.angle[1] = -40.0;
        joint.angle[2] = 400.0;
        joint.angle[3] = -1700.0;
        joint.angle[4] = 2748.0;
        joint.angle[5] = -1300.0;
        // joint.angle[0] = 0.0;
        // joint.angle[1] = 0.0;
        // joint.angle[2] = 0.0;
        // joint.angle[3] = 0.0;
        // joint.angle[4] = 0.0;
        // joint.angle[5] = 0.0;
        joint.vel[0] = 400.0;
        joint.vel[1] = 200.0;
        joint.vel[2] = 400.0;
        joint.vel[3] = 400.0;
        joint.vel[4] = 400.0;
        joint.vel[5] = 200.0;
        for (int i = 0; i < 10; ++i) {
            joint_pub_->publish(joint);
            rclcpp::sleep_for(std::chrono::milliseconds(10));  // 少し間隔を入れるのが安全
        }
        rclcpp::sleep_for(std::chrono::seconds(2));
        result->success = true;
        result->error_string = "success"; // 成功メッセージを設定
        goal_handle->succeed(result); // ゴール成功を通知
        RCLCPP_INFO(this->get_logger(), "Action succeeded");
  
        //////////////////////////////////////////////////


    }

    
};
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SearchServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
