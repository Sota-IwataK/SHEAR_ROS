#include "rclcpp/rclcpp.hpp"
#include <string>
#include <chrono>
#include <memory>
#include <functional>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
using std::placeholders::_1;
using namespace std::chrono_literals;

using namespace std::chrono_literals;

using Response = rcl_interfaces::srv::SetParameters_Response::SharedPtr;


class GetGlobalParam : public rclcpp::Node
{
public:

    GetGlobalParam() : Node("robot_manager",
                                   rclcpp::NodeOptions()
                                       .allow_undeclared_parameters(true)
                                       .automatically_declare_parameters_from_overrides(true)
                                       )
    {
        RCLCPP_INFO(this->get_logger(), "%s",this->get_name());
        // parameters_client = this->create_client<rcl_interfaces::srv::GetParameters>("/global_parameter_server/get_parameters");
        parameters_client = std::make_shared<rclcpp::AsyncParametersClient>(this, "/global_parameter_server");
        request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
        param_pub_ = create_publisher<rcl_interfaces::msg::ParameterEvent>("/global_parameter_server/update_param", 1);
       
        // this->declare_parameter<std::string>("robot_name", "");
        rclcpp::Parameter my_robot_name_param_ = this->get_parameter("robot_name");
        my_name_ = my_robot_name_param_.as_string();
        robot_list_ = {my_robot_name_param_.as_string()};
        // this->set_parameter(rclcpp::Parameter("robot_list",new_list));
        

        rclcpp::Parameter robot_list_param;
        // std::vector<std::string> request_list = {"robot_list"};
        request->names.push_back("robot_list");

        
        rclcpp::WallRate rate(1s);
    
        while(rclcpp::ok())
        {
            check_robot_list();
            // if(!registerd_) robot_list_publish();
            rate.sleep();
        }
    }

    // ~GetGlobalParam()
    // {
    //     RCLCPP_INFO(this->get_logger(), "delete my name");
    //     if(registerd_)
    //     {
    //         std::vector<std::string> robot_list = robot_list_;
    //         std::string my_name = my_name_;
    //         //除名
    //         robot_list.erase(std::remove_if(robot_list.begin(), robot_list.end(), 
    //                     [my_name](std::string &name){
    //                        return(name == my_name);
    //                    }), 
    //                    robot_list.end());
    //         robot_list_publish(robot_list);
    //     }
    // }
    

    void check_robot_list()
    {
        while(!parameters_client->wait_for_service(std::chrono::seconds(1)))
        {
            if(!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
                
            RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
        }
        
        auto parameters_future = parameters_client->get_parameters({"robot_list"});
        //結果を受信するまで待機
        if(rclcpp::spin_until_future_complete(this->get_node_base_interface(), parameters_future) != rclcpp::FutureReturnCode::SUCCESS){
            RCLCPP_ERROR(this->get_logger(), "Failed to get param list.");
            return;
        }

        auto result = parameters_future.get();
        auto param = result.at(0);
        // auto param_type_name = param.get_type_name();
        // RCLCPP_INFO(this->get_logger(), "type name: '%s'", param_type_name.c_str());
        // std::vector<std::string> receive_robot_list = param.as_string_array();
        // RCLCPP_INFO(this->get_logger(),"result size = %d",int(result.size()));

        if(param.get_type_name() == "string_array")//robot_listに誰か既に登録している場合
        {
            std::vector<std::string> robot_list = param.as_string_array();
            robot_list_ = robot_list;
            // RCLCPP_INFO(this->get_logger(),"list size = %d",int(robot_list.size()));
            // for(auto name : robot_list)
            // {
            //     RCLCPP_INFO(this->get_logger(), "Got global param: '%s'", name.c_str());
            // }

            //robot_listに自分が登録されているか確認
            std::string my_name;
            my_name = my_name_;
             auto itr = std::find_if(robot_list.begin(), robot_list.end(),
                       [my_name](std::string &name){
                           return(name==my_name);
                       } );
            if( itr == robot_list.end() )//自分が登録されていない場合
            {
                auto param_msg = rcl_interfaces::msg::ParameterEvent();
                param_msg.stamp = this->now();
                robot_list_.push_back(my_name);
                param_msg.changed_parameters.push_back(rclcpp::Parameter("robot_list",robot_list_).to_parameter_msg());
                param_pub_->publish(param_msg);
                RCLCPP_INFO(this->get_logger(), "register my_name publish!");
            }
            else registerd_ = true;
        }
        else if(param.get_type_name() == "not set")//robot_listに何も登録されていない場合は新規作成して自分を登録
        {
            robot_list_publish(robot_list_);
        }
    }

    void robot_list_publish(std::vector<std::string> robot_list)
    {
        auto param_msg = rcl_interfaces::msg::ParameterEvent();
        param_msg.changed_parameters.push_back(rclcpp::Parameter("robot_list",robot_list).to_parameter_msg());
        param_msg.stamp = this->now();
        param_pub_->publish(param_msg);
        RCLCPP_INFO(this->get_logger(), "new robot_list publish!");
    }

private:
    // rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr parameters_client;
    std::shared_ptr<rclcpp::AsyncParametersClient> parameters_client;
    
    rclcpp::TimerBase::SharedPtr timer_;
    bool registerd_ = false;//自分を登録しているか
    rclcpp::Parameter my_robot_name_param_;
    std::string my_name_;
    rcl_interfaces::srv::GetParameters_Request::SharedPtr request;
    Response response;
    std::vector<std::string> params_names_vector;
    rclcpp::Publisher<rcl_interfaces::msg::ParameterEvent>::SharedPtr param_pub_;
    std::vector<std::string> robot_list_;
};



class MyPosePub : public rclcpp::Node
{
    public:
        MyPosePub()
        : Node("tf2_frame_listener")
        {
            tf_buffer_ =
                std::make_unique<tf2_ros::Buffer>(this->get_clock());
            transform_listener_ =
                std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

            // Call on_timer function every second
            timer_ = this->create_wall_timer(
                1s, std::bind(&MyPosePub::on_timer, this));
        }

    private:
        void on_timer()
        {     
            std::string fromFrameRel = "world";
            std::string toFrameRel = "base_link";
            geometry_msgs::msg::TransformStamped transformStamped;
            try {
                transformStamped = tf_buffer_->lookupTransform(
                toFrameRel, fromFrameRel,
                tf2::TimePointZero);
            } catch (tf2::TransformException & ex) {
            RCLCPP_INFO(
                this->get_logger(), "Could not transform %s to %s: %s",
                toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
            return;
            }
            // std::cout << transformStamped.transform.translation.x << std::endl;
            // std::cout << transformStamped.transform.translation.y << std::endl;
            // std::cout << transformStamped.transform.translation.z << std::endl;
        }
        // std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        rclcpp::TimerBase::SharedPtr timer_{nullptr};
        std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<GetGlobalParam>();
    // rclcpp::WallRate rate(1s);
    // while (rclcpp::ok()) 
    // {
        
    //     rclcpp::spin_some(node);
    //     rate.sleep();
    // }
    // rclcpp::spin(std::make_shared<GetGlobalParam>());
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

// int main(int argc, char **argv)
// {
//     rclcpp::init(argc, argv);
//     rclcpp::NodeOptions options;
//     options.allow_undeclared_parameters(true);
//     options.automatically_declare_parameters_from_overrides(true);

//     auto node = std::make_shared<rclcpp::Node>("global_parameter_server", options);
//     
//     node->get_parameter("robot_list", robot_list);

//     rclcpp::spin(node);
//     rclcpp::shutdown();
//     return 0;
// }