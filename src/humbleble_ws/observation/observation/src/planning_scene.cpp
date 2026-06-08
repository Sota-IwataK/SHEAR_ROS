#include <rclcpp/rclcpp.hpp>
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include <sensor_msgs/msg/laser_scan.hpp>
// #include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <observation_msgs/msg/object.hpp>
#include <observation_msgs/srv/set_objects.hpp>
#include <observation_msgs/srv/get_objects.hpp>
#include <unordered_map> 


using std::placeholders::_1;
using namespace std::chrono_literals;




class PlanningScene : public rclcpp::Node
{
public:
  PlanningScene()
  : Node("planning_scene")
  {
    using std::placeholders::_1;
    using std::placeholders::_2;  
    set_service_server_ = this->create_service<observation_msgs::srv::SetObjects>("set_objects", std::bind(&PlanningScene::set_service_cb, this, _1, _2));
    get_service_server_ = this->create_service<observation_msgs::srv::GetObjects>("get_objects", std::bind(&PlanningScene::get_service_cb, this, _1, _2));

    /*テスト用オブジェクト*/
    observation_msgs::msg::Object test1;
    test1.id = "test1";
    test1.obj_group = "test";
    test1.collision_object.pose.position.x = 1;
    test1.collision_object.pose.position.y = 0.5;
    test1.msg_type = test1.COLLISION;
    objects_[test1.id] = test1;
  }

  /*SetObjectsのサービスコールバック*/
  void set_service_cb(const std::shared_ptr<observation_msgs::srv::SetObjects::Request> request,
          std::shared_ptr<observation_msgs::srv::SetObjects::Response>      response)
  {
    for(auto object : request->objects)
    {
      objects_[object.id] = object;
    }
    response->error_string = "success";
  }

  void get_service_cb(const std::shared_ptr<observation_msgs::srv::GetObjects::Request> request,
          std::shared_ptr<observation_msgs::srv::GetObjects::Response>      response)
  {
    RCLCPP_INFO(this->get_logger(), "get_objects_cb");
    if(request->command == request->ID)
    {
      for(auto id : request->id_set)
      {
        if(objects_.find(id) != objects_.end()) //  キーが含まれていたら返す
              response->objects.push_back(objects_[id]);
      }      
    }
    else if(request->command == request->GROUP)
    {
      response->objects = get_obj_by_group(request->id_set.front()); 
    }
    else if(request->command == request->ALL)
    {
      //  未実装
    }
    else if(request->command == request->ALL_COLLISION)
    {
      response->objects = get_all_collision(); 
    }
    else
      throw std::logic_error("Invalid request");
   
  }

  // /*Objectを保存*/
  // void update_obj(const observation_msgs::msg::Object& obj)
  // {
  //   //  idが一致する配列のインデックスを取得し更新する
  //   auto itr = std::find_if(objects_.begin(), objects_.end(),
  //                       [obj](observation_msgs::msg::Object &h){  //  []の中は条件式で使う変数を、()には配列要素の型を設定
  //                           return(h.id == obj.id);
  //                       } );

  //   if(itr == objects_.end() )  //  見つからなかった場合は追加する
  //   {
  //     RCLCPP_INFO(this->get_logger(), "InitSetObject");
  //     objects_.push_back(obj);
  //     // return;
  //   }
  //   else  //  見つかった場合は更新する
  //   {
  //     int index = std::distance(objects_.begin(), itr); //  indexを取得
  //     objects_[index] = obj;
  //     // objects_[index].orientation = obj.orientation;
  //   }
  // }

  // /*idからObjectを取得*/
  // observation_msgs::msg::Object get_pose_by_id(const std::string& object_id)
  // {
  //   observation_msgs::msg::Object result;
  //   auto itr = std::find_if(objects_.begin(), objects_.end(),
  //                       [object_id](observation_msgs::msg::Object &h){
  //                           return(h.id==object_id);
  //                       } );
  //   if( itr == objects_.end() )
  //   {
  //     RCLCPP_WARN(this->get_logger(), "not found %s", object_id.c_str());
  //     result.id = "None";
  //   }
  //   else result = *itr;
  //   //  std::cout << *itr << std::endl;
  //   return result;
  // }

  /*obj_groupから複数Objectを取得*/
  std::vector<observation_msgs::msg::Object> get_obj_by_group(const std::string& obj_group)
  {
    typedef std::unordered_map<std::string, observation_msgs::msg::Object>::value_type map_value_type;

    std::vector<observation_msgs::msg::Object> detected_objects;
    auto itr = std::find_if(objects_.begin(), objects_.end(),
                        [obj_group](const map_value_type &h){
                            return(h.second.obj_group==obj_group);
                        } );

    while (itr != objects_.end())
    {
      detected_objects.push_back(itr->second);
      itr = std::find_if(++itr, objects_.end(),
                          [obj_group](const map_value_type &h){
                              return(h.second.obj_group==obj_group);
                          } );
    }

    if(detected_objects.size()==0)
      RCLCPP_WARN(this->get_logger(), "not found %s", obj_group.c_str());
    // for(auto obj: detected_objects)
    // {
    //   RCLCPP_INFO(this->get_logger(), "found %s", obj.id.c_str());
    // }
    return detected_objects;
  }

  std::vector<observation_msgs::msg::Object> get_all_collision()
  {
    typedef std::unordered_map<std::string, observation_msgs::msg::Object>::value_type map_value_type;

    std::vector<observation_msgs::msg::Object> detected_objects;
    auto itr = std::find_if(objects_.begin(), objects_.end(),
                        [](const map_value_type &h){
                            return(h.second.msg_type==h.second.COLLISION);
                        } );


    while (itr != objects_.end())
    {
      detected_objects.push_back(itr->second);
      itr = std::find_if(++itr, objects_.end(),
                          [](const map_value_type &h){
                              return(h.second.msg_type==h.second.COLLISION);
                          } );
    }
    return detected_objects;
  }

private:
  rclcpp::Service<observation_msgs::srv::SetObjects>::SharedPtr set_service_server_;
  rclcpp::Service<observation_msgs::srv::GetObjects>::SharedPtr get_service_server_;
  // std::vector<observation_msgs::msg::Object> objects_;
  std::unordered_map<std::string, observation_msgs::msg::Object> objects_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;

  // auto node1 = std::make_shared<NavObservation>();
  // exec.add_node(node1);
  auto node2 = std::make_shared<PlanningScene>();
  exec.add_node(node2);

  exec.spin();



  // auto node = std::make_shared<PathSub>();
  // rclcpp::spin(node->get_node_base_interface());
  // rclcpp::spin(std::make_shared<RobotPoseSub>());
  // rclcpp::spin(std::make_shared<NavObservation>());

  // rclcpp::spin(std::make_shared<CloseToObstacle>());
  rclcpp::shutdown();
  return 0;
}