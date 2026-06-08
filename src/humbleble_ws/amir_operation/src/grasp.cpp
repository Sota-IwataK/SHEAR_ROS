#include "rclcpp/rclcpp.hpp" // ROS 2の基本的なNode機能を提供
#include "std_msgs/msg/empty.hpp" // 空のメッセージ型
#include "behavior_tree_msgs/action/grasp.hpp" //グラスプのためのカスタムアクション型
#include "geometry_msgs/msg/pose_stamped.hpp" 
#include "rclcpp_action/rclcpp_action.hpp" // アクションサーバーを作成するためのライブラリ
#include "amir_interfaces/msg/amir_cmd.hpp" //amirとのやり取りをするためのpub用型
#include "amir_interfaces/msg/amir_sensor.hpp" //amirとのやり取りをするためのsub用型
#include <math.h>
#include <std_msgs/msg/float32.hpp>
#include <stdio.h>

// graspクラスを定義
class grasp : public rclcpp::Node
{
public:
    // エイリアス定義でコードの可読性を向上
    using GraspAction = behavior_tree_msgs::action::Grasp;
    using GoalHandlegrasp = rclcpp_action::ServerGoalHandle<GraspAction>;

    // コンストラクタ: ノードの初期化とパブリッシャー、サブスクライバー、アクションサーバーの設定
    grasp()
        : Node("grasp_action_server") // ノード名を指定
    {

        // QoS 設定: BEST_EFFORT（データ損失を許容）
        rclcpp::QoS qos_profile(10); // キューサイズ10
        qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

        // rclcpp::QoS qos(rclcpp::KeepLast{10});
        // パブリッシャーを初期化: "/amir/grasp"トピックにコマンドを送信
        grasp_pub_ = this->create_publisher<amir_interfaces::msg::AmirCmd>("/motor_sub", 10);

        //グリッパーの値をパブリッシュ(btで受け取るため)
        gripper_pub_ = this->create_publisher<std_msgs::msg::Float32>("/gripper_value", 10);

        // サブスクライバーを初期化: 目標手先位置、オブジェクトの座標？
        grasp_sub_ = this->create_subscription<amir_interfaces::msg::AmirSensor>("/encoder_pub", qos_profile, std::bind(&grasp::angle_callback, this, std::placeholders::_1));
        // grasp_sub_ = this->create_subscription<amir_interfaces::msg::AmirSensor>(
        //     "/encoder_pub",qos,
        //     [this](const amir_interfaces::msg::AmirSensor::SharedPtr msg) -> void {angle_callback(std::move(msg));
        // });

        // アクションサーバーを初期化
        action_server_ = rclcpp_action::create_server<GraspAction>(
            this, // 現在のノードを指定
            "amir/bt_grasp", // アクション名
            std::bind(&grasp::handle_goal, this, std::placeholders::_1, std::placeholders::_2), // ゴール受付コールバック
            std::bind(&grasp::handle_cancel, this, std::placeholders::_1), // キャンセル受付コールバック
            std::bind(&grasp::handle_accepted, this, std::placeholders::_1)); // ゴール処理コールバック

        // アクションサーバーの起動ログを出力
        RCLCPP_INFO(this->get_logger(), "****** grasp action server started *****");
    }

private:
    // パブリッシャー: graspコマンドを送信
    rclcpp::Publisher<amir_interfaces::msg::AmirCmd>::SharedPtr grasp_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr gripper_pub_;
    // サブスクライバー: 目標手先位置、オブジェクトの座標？を取得
    rclcpp::Subscription<amir_interfaces::msg::AmirSensor>::SharedPtr grasp_sub_;
  
    // アクションサーバー
    rclcpp_action::Server<GraspAction>::SharedPtr action_server_;

        double current_angle1;
        double current_angle2;
        double current_angle3;
        double current_angle4;
        double current_angle5;
        double current_angle6;
        std::string id;

    // ゴールを受け付けた際の処理
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &uuid, // ゴールのUUID
        std::shared_ptr<const GraspAction::Goal> goal) // ゴールの内容
    {
        RCLCPP_INFO(this->get_logger(), "Received grasp goal request");
        (void)uuid; // この場合、UUIDは使用しない
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; // ゴールを受け付け、実行を開始
    }

    // ゴールのキャンセルリクエストを受け付けた際の処理
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandlegrasp> goal_handle) // ゴールハンドル
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle; // この場合、ゴールハンドルは使用しない
        return rclcpp_action::CancelResponse::ACCEPT; // キャンセルを受け付け
    }

    // ゴールを受け入れた後の処理
    void handle_accepted(const std::shared_ptr<GoalHandlegrasp> goal_handle)
    {
        // 別スレッドでアクションを実行（非同期処理）
        std::thread([this, goal_handle]() {
            auto goal = goal_handle->get_goal();
            
            id = goal->id;

            execute(goal_handle);
        }).detach();
    }

    //現在のangle[0]~angle[5]を取得
    void angle_callback(const amir_interfaces::msg::AmirSensor::SharedPtr msg)
    {
        //取得した各関節角をcurrent_angle1~5に入れる
        current_angle1 = msg->angle[0];
        current_angle2 = msg->angle[1];
        current_angle3 = msg->angle[2];
        current_angle4 = msg->angle[3];
        current_angle5 = msg->angle[4];
        current_angle6 = msg->angle[5];

        auto gripper_msg = std::make_shared<std_msgs::msg::Float32>();
        gripper_msg->data = current_angle6;

        gripper_pub_ -> publish(*gripper_msg);
    }


    // アクションの実行処理
    void execute(const std::shared_ptr<GoalHandlegrasp> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing grasp action");

        auto result = std::make_shared<GraspAction::Result>(); // 実行結果を格納する変数

        RCLCPP_INFO(this->get_logger(), "grasp start");

       auto joint =  amir_interfaces::msg::AmirCmd();

        //pickの場合
        if (id == "pick") {
            RCLCPP_INFO(this->get_logger(), "pick start");

            // angle[4]を0°に設定

            joint.angle[0] = current_angle1;
            joint.angle[1] = current_angle2;
            joint.angle[2] = current_angle3;
            joint.angle[3] = current_angle4;
            joint.angle[4] = current_angle5;
            joint.angle[5] = 0.0;

            joint.vel[0] = 200.0;
            joint.vel[1] = 200.0;
            joint.vel[2] = 200.0;
            joint.vel[3] = 200.0;
            joint.vel[4] = 400.0;
            joint.vel[5] = 200.0;
            for (int i = 0; i < 6; ++i) {
            grasp_pub_->publish(joint);
            }
            RCLCPP_INFO(this->get_logger(), "Publishing angle5 0° for pick");

            std::cout << "current_angle1 : " << current_angle1 << "current_angle6 : " << current_angle6 << std::endl;

            sleep(5);

            // 現在の角度を確認し、0°でなければ成功、0°なら失敗
            // rclcpp::Rate rate(1.0); //CPUの負荷を抑制するために1秒おきにループして角度を確認
            // while (rclcpp::ok()) {
            //     feedback->current_angle5 = current_angle5;
            //     goal_handle->publish_feedback(feedback);
            //     RCLCPP_INFO(this->get_logger(), "Current angle5: %f", current_angle5);

                if (current_angle6 != 0.0) {
                    RCLCPP_INFO(this->get_logger(), "pick ok");
                    result->error_string = "success"; // 成功メッセージを設定
                    result->success = true;
                    goal_handle->succeed(result); // ゴール成功を通知
                    return;
                }
                else if(current_angle6 == 0.0){
                    RCLCPP_WARN(this->get_logger(), "pick failed....");
                    result->error_string = "grasp_pick_failed"; // 失敗メッセージを設定
                    result->success = false;
                    goal_handle->succeed(result); // ゴール成功として終了（結果は失敗）

                }

            //     rate.sleep();
            // }

        //placeの場合
        } else if (id == "place") {
            // 角度を75°に設定し、成功を返す
            joint.angle[0] = current_angle1;
            joint.angle[1] = current_angle2;
            joint.angle[2] = current_angle3;
            joint.angle[3] = current_angle4;
            joint.angle[4] = current_angle5;
            joint.angle[5] = (-75.0/180.0)*M_PI*1000;

            joint.vel[0] = 200.0;
            joint.vel[1] = 200.0;
            joint.vel[2] = 200.0;
            joint.vel[3] = 200.0;
            joint.vel[4] = 400.0;
            joint.vel[5] = 200.0;

            for (int i = 0; i < 6; ++i) {
            grasp_pub_->publish(joint);
            }
            RCLCPP_INFO(this->get_logger(), "Publishing angle5 -75° for place");
            sleep(5);
            RCLCPP_INFO(this->get_logger(), "place ok");
            result->error_string = "success"; // 成功メッセージを設定
            result->success = true;
            goal_handle->succeed(result); // ゴール成功を通知
        }
        
        RCLCPP_INFO(this->get_logger(), "grasp finish");
    }

};

// メイン関数: ノードを起動
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv); // ROS 2の初期化
    auto node = std::make_shared<grasp>(); // graspノードを作成
    rclcpp::spin(node); // ノードを実行
    rclcpp::shutdown(); // ROS 2の終了処理
    return 0;
}
