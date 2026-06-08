#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "behavior_tree_msgs/action/pick.hpp"
#include "behavior_tree_msgs/action/before_arm.hpp"
#include "behavior_tree_msgs/action/after_arm.hpp"
#include "behavior_tree_msgs/action/grasp.hpp"
#include "behavior_tree_msgs/action/place.hpp"
#include <amir_interfaces/msg/amir_cmd.hpp>
#include "amir_interfaces/msg/amir_sensor.hpp" 



using PickAction = behavior_tree_msgs::action::Pick;
using BeforeArmAction = behavior_tree_msgs::action::BeforeArm;
using AfterArmAction = behavior_tree_msgs::action::AfterArm;
using GraspAction = behavior_tree_msgs::action::Grasp;
// using PlaceAction = behavior_tree_msgs::action::Place;

class PickServer : public rclcpp::Node
{
public:
    PickServer() : Node("pick_server")
    {
        // QoS 設定: BEST_EFFORT（データ損失を許容）
        rclcpp::QoS qos_profile(10); // キューサイズ10
        qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

        // Pickアクションサーバーを作成
        action_server_ = rclcpp_action::create_server<PickAction>(
            this,
            "pick",
            std::bind(&PickServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&PickServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&PickServer::handle_accepted, this, std::placeholders::_1));

        // BeforeArmアクションクライアントを作成
        before_arm_client_ = rclcpp_action::create_client<BeforeArmAction>(this, "amir/bt_before_arm");
        // AfterArmアクションクライアントを作成
        after_arm_client_ = rclcpp_action::create_client<AfterArmAction>(this, "amir/bt_after_arm");
        // Graspアクションクライアントを作成
        grasp_client_ = rclcpp_action::create_client<GraspAction>(this, "amir/bt_grasp");
        // Placeアクションクライアントを作成
        // place_client_ = rclcpp_action::create_client<PlaceAction>(this, "/place");
        // 関節角のパブリッシャー
        joint_pub_ = this->create_publisher<amir_interfaces::msg::AmirCmd>("/motor_sub", 10);
        // 関節角のサブスクライバー
        joint_sub_ = this->create_subscription<amir_interfaces::msg::AmirSensor>("/encoder_pub", qos_profile, std::bind(&PickServer::angle_callback, this, std::placeholders::_1));
        // アクションサーバーの起動ログを出力
        RCLCPP_INFO(this->get_logger(), "****** Pick action server started ******");

        //アームを開いた状態にする
        // gripper_open();
        
    }

private:
    rclcpp_action::Server<PickAction>::SharedPtr action_server_;
    rclcpp_action::Client<BeforeArmAction>::SharedPtr before_arm_client_;
    rclcpp_action::Client<AfterArmAction>::SharedPtr after_arm_client_;
    rclcpp_action::Client<GraspAction>::SharedPtr grasp_client_;
    // rclcpp_action::Client<PlaceAction>::SharedPtr place_client_;
    rclcpp::Publisher<amir_interfaces::msg::AmirCmd>::SharedPtr joint_pub_;
    rclcpp::Subscription<amir_interfaces::msg::AmirSensor>::SharedPtr joint_sub_;

    //encoder_pubから取得した各関節角
    double current_angle1;
    double current_angle2;
    double current_angle3;
    double current_angle4;
    double current_angle5;
    double current_angle6;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const PickAction::Goal> goal)
    {
        // RCLCPP_INFO(this->get_logger(), "Received pick goal: x=%f, y=%f, z=%f", goal->x, goal->y, goal->z);
        // RCLCPP_INFO(this->get_logger(),
        //             "Received pick goal:\n"
        //             "Position: x=%f, y=%f, z=%f\n"
        //             "Orientation: x=%f, y=%f, z=%f, w=%f",
        //             goal->pose1.pose.position.x,
        //             goal->pose1.pose.position.y,
        //             goal->pose1.pose.position.z,
        //             goal->pose1.pose.orientation.x,
        //             goal->pose1.pose.orientation.y,
        //             goal->pose1.pose.orientation.z,
        //             goal->pose1.pose.orientation.w);
        
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<PickAction>> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Cancel request received");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<PickAction>> goal_handle)
    {
        std::thread([this, goal_handle]() {
            this->execute(goal_handle);
        }).detach();
    }

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
        // RCLCPP_INFO(this->get_logger(), "current_angle6:%f", current_angle1);
    }

    void grasp(const std::shared_ptr<rclcpp_action::ServerGoalHandle<PickAction>> goal_handle)
    {
        auto goal = goal_handle->get_goal();
        auto result = std::make_shared<PickAction::Result>();
        
        GraspAction::Goal grasp_goal;
        grasp_goal.id = "pick";

        // Graspアクションサーバーが利用可能かどうかを確認
        if (!grasp_client_->wait_for_action_server(std::chrono::seconds(5)))
        {
            RCLCPP_ERROR(this->get_logger(), "Grasp action server not available");

            return;
        }

        auto send_goal_result = rclcpp_action::Client<GraspAction>::SendGoalOptions();

        send_goal_result.result_callback = [this, goal_handle, result](const rclcpp_action::ClientGoalHandle<GraspAction>::WrappedResult &grasp_result){
            if (grasp_result.code == rclcpp_action::ResultCode::SUCCEEDED)
            {
                // success フィールドを確認
                // graspが成功したら、持ち上げる
                if (grasp_result.result->success)
                {
                    // AfterArmアクションのゴールを作成
                    AfterArmAction::Goal after_arm_goal;

                    //持ち上げるときの目標位置設定
                    float lift_obj_opsition_x,lift_obj_opsition_y,lift_obj_opsition_z;

                    lift_obj_opsition_x = 0.30;
                    lift_obj_opsition_y = 0.0;
                    lift_obj_opsition_z = 0.60;

                    after_arm_goal.pose_obj.pose.position.x = lift_obj_opsition_x;
                    after_arm_goal.pose_obj.pose.position.y = lift_obj_opsition_y;
                    after_arm_goal.pose_obj.pose.position.z = lift_obj_opsition_z;

                    // AfterArmアクションサーバーが利用可能かどうかを確認
                    if (!after_arm_client_->wait_for_action_server(std::chrono::seconds(5)))
                    {
                        RCLCPP_ERROR(this->get_logger(), "Arm action server not available");
                        return;
                    }

                    auto send_goal_result = rclcpp_action::Client<AfterArmAction>::SendGoalOptions();

                    // 結果コールバックを設定
                    send_goal_result.result_callback = [this, goal_handle, result](const rclcpp_action::ClientGoalHandle<AfterArmAction>::WrappedResult &after_arm_result){
                        // AfterArm サーバーからの結果を取得
                        if (after_arm_result.code == rclcpp_action::ResultCode::SUCCEEDED)
                        {
                            // success フィールドを確認
                            if (after_arm_result.result->success)
                            {
                                // RCLCPP_INFO(this->get_logger(), "AfterArm succeeded: %s", before_arm_result->error_string.c_str());
                                auto result = std::make_shared<PickAction::Result>();
                                RCLCPP_INFO(this->get_logger(), "Pick Successed");
                                result->error_string = "success"; // 成功メッセージを設定
                                goal_handle->succeed(result); // ゴール成功を通知

                                //Placeを起動
                                // place(goal_handle);
                            }
                            else
                            {
                                RCLCPP_ERROR(this->get_logger(), "AfterArm failed");
                            }
                        }
                    };

                    // AfterArm アクションサーバーで処理を実行．ゴールを非同期で送信．サーバーがゴール処理を完了すると、クライアントの result_callback が呼び出される
                    after_arm_client_->async_send_goal(after_arm_goal, send_goal_result);

                }
                else
                {
                    RCLCPP_ERROR(this->get_logger(), "AfterArm failed");
                }
            }
        };
        // Grasp アクションサーバーで処理を実行．ゴールを非同期で送信．サーバーがゴール処理を完了すると、クライアントの result_callback が呼び出される
        grasp_client_->async_send_goal(grasp_goal, send_goal_result);
    }
    void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<PickAction>> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing pick action");
        auto goal = goal_handle->get_goal();
        double x =  goal->pose_obj.pose.position.x;
        double y =  goal->pose_obj.pose.position.y;
        double z =  goal->pose_obj.pose.position.z;
        RCLCPP_INFO(this->get_logger(),
            "Pick action goal position: x=%.3f, y=%.3f, z=%.3f",
            x, y, z);
        auto result = std::make_shared<PickAction::Result>();
        // BeforeArmアクションのゴールを作成
        BeforeArmAction::Goal before_arm_goal;
        before_arm_goal.pose_obj.pose.position.x = goal->pose_obj.pose.position.x;
        before_arm_goal.pose_obj.pose.position.y = goal->pose_obj.pose.position.y;
        before_arm_goal.pose_obj.pose.position.z = goal->pose_obj.pose.position.z;
        // BeforeArmアクションサーバーが利用可能かどうかを確認
        if (!before_arm_client_->wait_for_action_server(std::chrono::seconds(5)))
        {
            RCLCPP_ERROR(this->get_logger(), "BeforeArm action server not available");
            return;
        }
        auto send_goal_result = rclcpp_action::Client<BeforeArmAction>::SendGoalOptions();
        // 結果コールバックを設定
        send_goal_result.result_callback = [this, goal_handle, result](const rclcpp_action::ClientGoalHandle<BeforeArmAction>::WrappedResult &before_arm_result){
            // BeforeArm サーバーからの結果を取得
            if (before_arm_result.code == rclcpp_action::ResultCode::SUCCEEDED)
            {
                // success フィールドを確認
                if (before_arm_result.result->success)
                {
                    RCLCPP_INFO(this->get_logger(), "BeforeArm succeeded");
                    // グラスプ処理を実行
                    grasp(goal_handle);
                }
                else
                {
                    RCLCPP_ERROR(this->get_logger(), "BeforeArm failed");
                }
            }
        };
        RCLCPP_ERROR(this->get_logger(), "BeforeArm action server");
        // BeforeArm アクションサーバーで処理を実行．ゴールを非同期で送信．サーバーがゴール処理を完了すると、クライアントの result_callback が呼び出される
        before_arm_client_->async_send_goal(before_arm_goal, send_goal_result);
    }
};
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PickServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
