#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "behavior_tree_msgs/action/pick.hpp"
#include "behavior_tree_msgs/action/move_arm.hpp"
#include "behavior_tree_msgs/action/grasp.hpp"
#include "behavior_tree_msgs/action/place.hpp"
#include <amir_interfaces/msg/amir_cmd.hpp>
#include "amir_interfaces/msg/amir_sensor.hpp" 



using PickAction = behavior_tree_msgs::action::Pick;
using MoveArmAction = behavior_tree_msgs::action::MoveArm;
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

        // MoveArmアクションクライアントを作成
        move_arm_client_ = rclcpp_action::create_client<MoveArmAction>(this, "amir/bt_move_arm");
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
    rclcpp_action::Client<MoveArmAction>::SharedPtr move_arm_client_;
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
        RCLCPP_INFO(this->get_logger(), "Received pick goal: x=%f, y=%f, z=%f", goal->x, goal->y, goal->z);
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

    // void gripper_open(const std::shared_ptr<rclcpp_action::ServerGoalHandle<PickAction>> goal_handle)
    // {
    //     auto goal = goal_handle->get_goal();
    //     auto result = std::make_shared<PickAction::Result>();
        
    //     GraspAction::Goal grasp_goal;
    //     grasp_goal.id = "place";
        
    //     RCLCPP_INFO(this->get_logger(), "Gripper Open");

    //     // Graspアクションサーバーが利用可能かどうかを確認
    //     if (!grasp_client_->wait_for_action_server(std::chrono::seconds(5)))
    //     {
    //         RCLCPP_ERROR(this->get_logger(), "Grasp action server not available");
    //         // result->success = false;
    //         // result->error_message = "MoveArm action server not available";
    //         // goal_handle->abort(result);
    //         return;
    //     }

    //     auto gripper_result = rclcpp_action::Client<GraspAction>::SendGoalOptions();
    //     grasp_client_->async_send_goal(grasp_goal, gripper_result);
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
        // RCLCPP_INFO(this->get_logger(), "current_angle6:%f", current_angle1);
    }


    // void place(const std::shared_ptr<rclcpp_action::ServerGoalHandle<PickAction>> goal_handle)
    // {
    //     RCLCPP_INFO(this->get_logger(), "Executing Place action");

    //     auto goal = goal_handle->get_goal();
    //     auto result = std::make_shared<PickAction::Result>();

    //     // Placeアクションのゴールを作成
    //     PlaceAction::Goal place_goal;

    //     //place位置
    //     float place_position_x,place_position_y,place_position_z;

    //     place_position_x = 0.45;
    //     place_position_y = -0.40;
    //     place_position_z = 0.40;
        
    //     place_goal.x = place_position_x;
    //     place_goal.y = place_position_y;
    //     place_goal.z = place_position_z;

    //     // MoveArmアクションサーバーが利用可能かどうかを確認
    //     if (!place_client_->wait_for_action_server(std::chrono::seconds(5)))
    //     {
    //         RCLCPP_ERROR(this->get_logger(), "Place action server not available");
    //         // result->success = false;
    //         // result->error_message = "MoveArm action server not available";
    //         // goal_handle->abort(result);
    //         return;
    //     }

    //     auto send_goal_result = rclcpp_action::Client<PlaceAction>::SendGoalOptions();

    //     RCLCPP_ERROR(this->get_logger(), "Place action server");

    //     // MoveArm アクションサーバーで処理を実行．ゴールを非同期で送信．サーバーがゴール処理を完了すると、クライアントの result_callback が呼び出される
    //     place_client_->async_send_goal(place_goal, send_goal_result);
    // }

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
            // result->success = false;
            // result->error_message = "MoveArm action server not available";
            // goal_handle->abort(result);
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
                    // RCLCPP_INFO(this->get_logger(), "MoveArm succeeded: %s", move_arm_result->error_string.c_str());
                    // MoveArmアクションのゴールを作成
                    MoveArmAction::Goal move_arm_goal;

                    //持ち上げるときの目標位置設定
                    float lift_obj_opsition_x,lift_obj_opsition_y,lift_obj_opsition_z;

                    lift_obj_opsition_x = 0.30;
                    lift_obj_opsition_y = 0.0;
                    lift_obj_opsition_z = 0.60;

                    move_arm_goal.x = lift_obj_opsition_x;
                    move_arm_goal.y = lift_obj_opsition_y;
                    move_arm_goal.z = lift_obj_opsition_z;

                    // MoveArmアクションサーバーが利用可能かどうかを確認
                    if (!move_arm_client_->wait_for_action_server(std::chrono::seconds(5)))
                    {
                        RCLCPP_ERROR(this->get_logger(), "MoveArm action server not available");
                        // result->success = false;
                        // result->error_message = "MoveArm action server not available";
                        // goal_handle->abort(result);
                        return;
                    }

                    auto send_goal_result = rclcpp_action::Client<MoveArmAction>::SendGoalOptions();

                    // 結果コールバックを設定
                    send_goal_result.result_callback = [this, goal_handle, result](const rclcpp_action::ClientGoalHandle<MoveArmAction>::WrappedResult &move_arm_result){
                        // MoveArm サーバーからの結果を取得
                        if (move_arm_result.code == rclcpp_action::ResultCode::SUCCEEDED)
                        {
                            // success フィールドを確認
                            if (move_arm_result.result->success)
                            {
                                // RCLCPP_INFO(this->get_logger(), "MoveArm succeeded: %s", move_arm_result->error_string.c_str());
                                auto result = std::make_shared<PickAction::Result>();
                                RCLCPP_INFO(this->get_logger(), "Pick Successed");
                                result->error_string = "success"; // 成功メッセージを設定
                                goal_handle->succeed(result); // ゴール成功を通知

                                //Placeを起動
                                // place(goal_handle);
                            }
                            else
                            {
                                RCLCPP_ERROR(this->get_logger(), "MoveArm failed");
                            }
                        }
                    };

                    // MoveArm アクションサーバーで処理を実行．ゴールを非同期で送信．サーバーがゴール処理を完了すると、クライアントの result_callback が呼び出される
                    move_arm_client_->async_send_goal(move_arm_goal, send_goal_result);

                }
                else
                {
                    RCLCPP_ERROR(this->get_logger(), "MoveArm failed");
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
        auto result = std::make_shared<PickAction::Result>();
        // MoveArmアクションのゴールを作成
        MoveArmAction::Goal move_arm_goal;
        // move_arm_goal.x = goal->pose1.pose.position.x;
        // move_arm_goal.y = goal->pose1.pose.position.y;
        // move_arm_goal.z = goal->pose1.pose.position.z;
        move_arm_goal.x = goal->x;
        move_arm_goal.y = goal->y;
        move_arm_goal.z = goal->z;
        // MoveArmアクションサーバーが利用可能かどうかを確認
        if (!move_arm_client_->wait_for_action_server(std::chrono::seconds(5)))
        {
            RCLCPP_ERROR(this->get_logger(), "MoveArm action server not available");
            // result->success = false;
            // result->error_message = "MoveArm action server not available";
            // goal_handle->abort(result);
            return;
        }
        auto send_goal_result = rclcpp_action::Client<MoveArmAction>::SendGoalOptions();
        // 結果コールバックを設定
        send_goal_result.result_callback = [this, goal_handle, result](const rclcpp_action::ClientGoalHandle<MoveArmAction>::WrappedResult &move_arm_result){
            // MoveArm サーバーからの結果を取得
            if (move_arm_result.code == rclcpp_action::ResultCode::SUCCEEDED)
            {
                // success フィールドを確認
                if (move_arm_result.result->success)
                {
                    RCLCPP_INFO(this->get_logger(), "MoveArm succeeded");
                    // グラスプ処理を実行
                    grasp(goal_handle);
                }
                else
                {
                    RCLCPP_ERROR(this->get_logger(), "MoveArm failed");
                }
            }
        };
        RCLCPP_ERROR(this->get_logger(), "MoveArm action server");
        // MoveArm アクションサーバーで処理を実行．ゴールを非同期で送信．サーバーがゴール処理を完了すると、クライアントの result_callback が呼び出される
        move_arm_client_->async_send_goal(move_arm_goal, send_goal_result);
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
