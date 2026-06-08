#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <iostream>
// #include <opencv2/opencv.hpp>
// #include <opencv2/core/core.hpp>
// #include <opencv2/imgproc/imgproc.hpp>
// #include <opencv2/highgui/highgui.hpp>
// #include <opencv2/objdetect/objdetect.hpp>
#include <cv_bridge/cv_bridge.h>
// #include <sensor_msgs/Image.h>
// #include <XnOpenNI.h>
// #include <XnCodecIDs.h>
// #include <XnCppWrapper.h>
// #include <XnPropNames.h>
// #include <pcl/visualization/cloud_viewer.h>

#include <vector>
#include <string>
// #include <cv.h>
// #include <highgui.h>
#include <sstream>
// #include <std_msgs/String.h>

// #include <std_msgs/Float64MultiArray.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/LU>

// #include <youbot_projected_gradient/pixel.h>
// #include <youbot_projected_gradient/from_frame.h>
// #include <youbot_projected_gradient/object.h>
// #include <youbot_projected_gradient/object_array.h>

// #include <sensor_msgs/PointCloud2.h>

// #define BOOST_BIND_NO_PLACEHOLDERS  //pclより前に書く
// #include <pcl_conversions/pcl_conversions.h>
// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>
#include <vector>
#include "geometry_msgs/msg/point.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <darknet_ros_msgs/msg/bounding_boxes.hpp>
#include <darknet_ros_msgs/msg/bounding_box.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>

// #include <darknet_ros_msgs/msg/CheckForObjectsAction.hpp>

using namespace Eigen;
using namespace cv;
// using namespace std;

using std::placeholders::_1;
using namespace std::chrono_literals;

class Object_callback : public rclcpp::Node
{
public:
  Object_callback();

  // private:
  rclcpp::Subscription<darknet_ros_msgs::msg::BoundingBoxes>::SharedPtr sub_boundingboxes_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_;
  rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr pub_object_pose;
  
	// ros::Subscriber sub_image;
	// ros::Publisher pub_object_pose;

	bool show_image_window_object_callback;

  bool depth_is_subscribed;

  // pcl::PointCloud<pcl::PointXYZ> depth;

  int sampling_pixel_step;
	int sampling_num;//n^2をいれること

  int cols, rows;

	std::vector<Vector2f> pixel_bounding_box;
	std::vector<std::vector<Vector2f> > pixel_bounding_boxes_1, pixel_bounding_boxes_2;
	Vector3f eigenvector;

	// youbot_projected_gradient::object object;
  // youbot_projected_gradient::object_array msg;
  vision_msgs::msg::Detection3DArray objects;

  std::string target_label_1, target_label_2;
  rclcpp::TimerBase::SharedPtr timer_;

	// void execute_eigenvector(const youbot_projected_gradient::pixel& pixel_top_left, const youbot_projected_gradient::pixel& pixel_bottom_right, const pcl::PointCloud<pcl::PointXYZ>& depth, const double& distance);

  
	void boundingbox_callback(const std::shared_ptr<darknet_ros_msgs::msg::BoundingBoxes> boxes);
  void cloud_callback(const std::shared_ptr<sensor_msgs::msg::PointCloud2> input);
	// void cloud_callback(const sensor_msgs::PointCloud2ConstPtr& input);
	// void show_image_callback(const sensor_msgs::ImageConstPtr& image);
  void show_image_callback(const std::shared_ptr<sensor_msgs::msg::Image> image);
  void timer_callback();
};

Object_callback::Object_callback()
: Node("dummy_obj_pub")
{
  depth_is_subscribed = false;

  sampling_pixel_step = 3;
	sampling_num = pow(3.0, 2.0);//n^2をいれること
  objects.header.frame_id = "camera_link";

  pub_object_pose = this->create_publisher<vision_msgs::msg::Detection3DArray>("object_callback", 10);

  timer_ = this->create_wall_timer(
    500ms, std::bind(&Object_callback::timer_callback, this));
}


void Object_callback::timer_callback()
{
  objects.detections.clear();

  
  vision_msgs::msg::Detection3D object;
  object.header.stamp = this->get_clock()->now();
  object.header.frame_id = "map";
  object.bbox.center.position.x = -3;//カメラ座標系x軸
  object.bbox.center.position.y = 1.5;//カメラ座標系y軸
  object.bbox.center.position.z = 0;//カメラ座標系z軸
  object.bbox.center.orientation.w = 1; //  仮 

  vision_msgs::msg::ObjectHypothesisWithPose obj_pose;  //候補
  obj_pose.hypothesis.class_id = "robot2"; 
  obj_pose.hypothesis.score = 1;
  obj_pose.pose.pose = object.bbox.center;

  object.results.push_back(obj_pose); //候補を追加

  objects.detections.push_back(object);
  pub_object_pose->publish(objects);
}


int main (int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Object_callback>());
  rclcpp::shutdown();

  // ros::init (argc,argv,"object_position_boundingbox");
  // ros::NodeHandle nh;

	// Object_callback object_callback;
	// ros::Rate loop_rate(50);

  // rclcpp::WallRate rate(1s);

  // while (rclcpp::ok()) {
  //   publisher->publish(greeting);
  //   rclcpp::spin_some(node);
  //   rate.sleep();
  // }

  return 0;
}
