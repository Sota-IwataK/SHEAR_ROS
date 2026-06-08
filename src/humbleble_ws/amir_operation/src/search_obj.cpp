#include "rclcpp/rclcpp.hpp" // ROS 2の基本的なNode機能を提供
#include "std_msgs/msg/empty.hpp" // 空のメッセージ型（着陸コマンド用）
#include "amir_interfaces/msg/amir_cmd.hpp" // アーム制御用
#include "amir_interfaces/msg/amir_sensor.hpp" // アーム制御用
#include "behavior_tree_msgs/action/search_obj.hpp" // カスタムアクション型
#include "rclcpp_action/rclcpp_action.hpp" // アクションサーバー
// #include "yolo_interfaces/msg/detection_result.hpp" // YOLOの結果
#include <math.h>

class SearchObj : public rclcpp::Node
{
public:
    using SearchObjAction = behavior_tree_msgs::action::SearchObj;
    using GoalHandleSearchObj = rclcpp_action::ServerGoalHandle<SearchObjAction>;

    SearchObj()
        : Node("SearchObj_action_server")
    {
        // アクションサーバーを初期化
        action_server_ = rclcpp_action::create_server<SearchObjAction>(
            this,
            "amir/bt_search_obj",
            std::bind(&SearchObj::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&SearchObj::handle_cancel, this, std::placeholders::_1),
            std::bind(&SearchObj::handle_accepted, this, std::placeholders::_1));

        // パブリッシャーを作成（アーム制御）
        arm_publisher_ = this->create_publisher<amir_interfaces::msg::AmirCmd>("/motor_sub", 10);

        // YOLOの結果を受け取るサブスクライバー
        // yolo_subscriber_ = this->create_subscription<yolo_interfaces::msg::DetectionResult>(
        //     "/yolo_detections",
        //     10,
        //     std::bind(&SearchObj::yolo_callback, this, std::placeholders::_1));

        arm_subscriber_ = this->create_subscription<amir_interfaces::msg::AmirSensor>(
            "/encoder_pub", 
            10, 
            std::bind(&SearchObj::arm_callback, this, std::placeholders::_1));


        RCLCPP_INFO(this->get_logger(), "****** SearchObj action server started *****");
    }

private:
    rclcpp_action::Server<SearchObjAction>::SharedPtr action_server_;
    rclcpp::Publisher<amir_interfaces::msg::AmirCmd>::SharedPtr arm_publisher_;
    // rclcpp::Subscription<yolo_interfaces::msg::DetectionResult>::SharedPtr yolo_subscriber_;
    rclcpp::Subscription<amir_interfaces::msg::AmirSensor>::SharedPtr arm_subscriber_;

    std::string detected_object_id_; // YOLOで検出されたオブジェクトID
    std::mutex detection_mutex_;

    double current_angle1;
    double current_angle2;
    double current_angle3;
    double current_angle4;
    double current_angle5;
    double current_angle6;

    void arm_callback(const amir_interfaces::msg::AmirSensor::SharedPtr msg)
    {
        //取得した各関節角をcurrent_angle1~5に入れる
        current_angle1 = msg->angle[0];
        current_angle2 = msg->angle[1];
        current_angle3 = msg->angle[2];
        current_angle4 = msg->angle[3];
        current_angle5 = msg->angle[4];
        current_angle6 = msg->angle[5];

    }

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const SearchObjAction::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received SearchObj goal request with object_id: %s", goal->object_id.c_str());
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleSearchObj> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleSearchObj> goal_handle)
    {
        std::thread([this, goal_handle]() {
            auto goal = goal_handle->get_goal();
            this->execute(goal_handle, goal->object_id);
        }).detach();
    }

    void execute(const std::shared_ptr<GoalHandleSearchObj> goal_handle, const std::string &object_id)
    {
        RCLCPP_INFO(this->get_logger(), "Executing SearchObj action");

        // アームを動かすメッセージを作成
        amir_interfaces::msg::AmirCmd arm_cmd;
        arm_cmd.angle[0] = 0; // 初期値 (mrad)
        arm_cmd.vel[0] = 10.0; // 初期速度

        detected_object_id_ = "bottle";

        bool moving_forward = true;

        // アクション実行中フラグ
        rclcpp::Rate rate(10);
        auto result = std::make_shared<SearchObjAction::Result>();

        const int max_angle_mrad = (170 * 1000 * M_PI) / 180; // 170度をmradに変換
        const int min_angle_mrad = -(170 * 1000 * M_PI) / 180; // -170度をmradに変換

        while (rclcpp::ok())
        {
            {
                std::lock_guard<std::mutex> lock(detection_mutex_);
                if (detected_object_id_ == object_id)
                {
                    RCLCPP_INFO(this->get_logger(), "Found object with id: %s", object_id.c_str());
                    // 現在のangleを保持
                    arm_cmd.angle[0] = current_angle1;
                    arm_cmd.angle[1] = current_angle2;
                    arm_cmd.angle[2] = current_angle3;
                    arm_cmd.angle[3] = current_angle4;
                    arm_cmd.angle[4] = current_angle5;
                    arm_cmd.vel[0] = 0.0; // STOP
                    result->error_string = "success"; // 成功メッセージを設定
                    result->success = true;
                    goal_handle->succeed(result);
                    arm_publisher_->publish(arm_cmd);
                    RCLCPP_INFO(this->get_logger(), "c");
                    return;
                }
            }

            // アームを動かす
            if (moving_forward)
            {
                arm_cmd.angle[0] += 100; // 100 mrad 増加
                if (arm_cmd.angle[0] >= max_angle_mrad)
                {
                    // 現在のangleを保持
                    arm_cmd.angle[1] = current_angle2;
                    arm_cmd.angle[2] = current_angle3;
                    arm_cmd.angle[3] = current_angle4;
                    arm_cmd.angle[4] = current_angle5;
                    arm_cmd.vel[0] = -10.0; // 逆方向
                    moving_forward = false;
                    RCLCPP_INFO(this->get_logger(), "a");

                }
            }
            else
            {
                arm_cmd.angle[0] -= 100; // 100 mrad 減少
                if (arm_cmd.angle[0] <= min_angle_mrad)
                {
                    // 現在のangleを保持
                    arm_cmd.angle[1] = current_angle2;
                    arm_cmd.angle[2] = current_angle3;
                    arm_cmd.angle[3] = current_angle4;
                    arm_cmd.angle[4] = current_angle5;
                    arm_cmd.vel[0] = 10.0; // 順方向
                    moving_forward = true;
                    RCLCPP_INFO(this->get_logger(), "b");
                }
            }
            arm_publisher_->publish(arm_cmd);

            rate.sleep();
        }

        // タイムアウトまたはエラー時の処理
        RCLCPP_INFO(this->get_logger(), "Failed to find object with id: %s", object_id.c_str());
        result->success = false;
        goal_handle->succeed(result);
    }

    // void yolo_callback(const yolo_interfaces::msg::DetectionResult::SharedPtr msg)
    // {
    //     std::lock_guard<std::mutex> lock(detection_mutex_);
    //     if (!msg->detections.empty())
    //     {
    //         detected_object_id_ = msg->detections[0].id; // 最初の検出結果を使用
    //     }
    // }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SearchObj>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
