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

#define BOOST_BIND_NO_PLACEHOLDERS  //pclより前に書く
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>
#include "geometry_msgs/msg/point.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "darknet_ros_msgs/msg/bounding_boxes.hpp"
#include "darknet_ros_msgs/msg/bounding_box.hpp"
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>


// #include <darknet_ros_msgs/msg/CheckForObjectsAction.hpp>

using namespace Eigen;
// using namespace cv;
// using namespace std;

using std::placeholders::_1;

class Object_callback : public rclcpp::Node
{
public:
  Object_callback();

private:
  rclcpp::Subscription<darknet_ros_msgs::msg::BoundingBoxes>::SharedPtr sub_boundingboxes_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_;
  rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr pub_object_pose_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_marksers_;
  
	// ros::Subscriber sub_image;
	// ros::Publisher pub_object_pose_;

	bool show_image_window_object_callback;

  bool depth_is_subscribed_;

  pcl::PointCloud<pcl::PointXYZ> depth;

  int sampling_pixel_step;
	int sampling_num;//n^2をいれること

  int cols_, rows_;

	std::vector<Vector2f> pixel_bounding_box;
	std::vector<std::vector<Vector2f> > pixel_bounding_boxes_1, pixel_bounding_boxes_2;
	Vector3f eigenvector;

	// youbot_projected_gradient::object object;
  // youbot_projected_gradient::object_array msg;
  vision_msgs::msg::Detection3DArray objects;
  std::string target_label_1, target_label_2;

	// void execute_eigenvector(const youbot_projected_gradient::pixel& pixel_top_left, const youbot_projected_gradient::pixel& pixel_bottom_right, const pcl::PointCloud<pcl::PointXYZ>& depth, const double& distance);

  
	void boundingbox_callback(const std::shared_ptr<darknet_ros_msgs::msg::BoundingBoxes> boxes);
  void cloud_callback(const std::shared_ptr<sensor_msgs::msg::PointCloud2> input);
  void show_image_callback(const std::shared_ptr<sensor_msgs::msg::Image> image);

  std::string camera_pointcloud_topic_;
  std::string camera_rgb_topic_;
  std::string bounding_boxes_topic_;
};

Object_callback::Object_callback()
: Node("object_position_boundingbox")
{
  if (!this->has_parameter("show_image_window_object_callback")) this->declare_parameter<bool>("show_image_window_object_callback", true);
  if (!this->has_parameter("camera_pointcloud_topic")) this->declare_parameter<std::string>("camera_pointcloud_topic", "/camera/depth_registered/points");  //depth_image_proc point_cloud_xyzrgbで処理しておくこと！
  if (!this->has_parameter("camera_rgb_topic")) this->declare_parameter<std::string>("camera_rgb_topic", "camera_rgb_topic");
  if (!this->has_parameter("bounding_boxes_topic")) this->declare_parameter<std::string>("bounding_boxes_topic", "darknet_ros/bounding_boxes");
  this->get_parameter("show_image_window_object_callback", show_image_window_object_callback);
  this->get_parameter("camera_pointcloud_topic", camera_pointcloud_topic_);
  this->get_parameter("camera_rgb_topic", camera_rgb_topic_);
  this->get_parameter("bounding_boxes_topic", bounding_boxes_topic_);
  depth_is_subscribed_ = false;

  objects.header.frame_id = "camera_link"; //pointcloudのフレーム
    
  sampling_pixel_step = 3;
	sampling_num = pow(3.0, 2.0);//n^2をいれること
 
	
  sub_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      camera_pointcloud_topic_, 1,
      std::bind(&Object_callback::cloud_callback, this, _1));

  // sub_image_ = this->create_subscription<sensor_msgs::msg::Image>(
  //     camera_rgb_topic_, rclcpp::SensorDataQoS(),
  //     std::bind(&Object_callback::show_image_callback, this, _1));

	
  sub_boundingboxes_ = this->create_subscription<darknet_ros_msgs::msg::BoundingBoxes>(
      bounding_boxes_topic_, rclcpp::SensorDataQoS(),
      std::bind(&Object_callback::boundingbox_callback, this, _1));//YOLO

  pub_object_pose_ = this->create_publisher<vision_msgs::msg::Detection3DArray>("object_callback", 10);
  pub_marksers_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("object_marker", 10);
  // sub_cloud= nh.subscribe("/camera_pointcloud_topic", 1, &Object_callback::cloud_callback, this);
  // sub_image = nh.subscribe("/camera_rgb_topic", 1, &Object_callback::show_image_callback, this);
	// pub_object_pose_ = nh.advertise<youbot_projected_gradient::object_array>("/object_callback", 10);
}

// void Object_callback::execute_eigenvector(const youbot_projected_gradient::pixel& pixel_top_left, const youbot_projected_gradient::pixel& pixel_bottom_right, const pcl::PointCloud<pcl::PointXYZ>& depth, const double& distance)
// {
//   // ROS_INFO("execute_eigenvector");

// 	std::vector<pcl::PointXYZ> point_cloud;
//   pcl::PointXYZ point;

// 	point_cloud.clear();

// 	for(int i=pixel_top_left.x; i<pixel_bottom_right.x; i++){
// 		for(int j=pixel_top_left.y; j<pixel_bottom_right.y; j++){
// 			if(depth.at(i,j).z < distance + 0.1){
//         point.x = depth.at(i,j).y;
//         point.y = -depth.at(i,j).x;
//         point.z = depth.at(i,j).z;
// 			  point_cloud.push_back(point);
// 			}
// 		}
// 	}

// 	pcl::PointXYZ point_cloud_sum, point_cloud_ave;

// 	for(int i=0; i<point_cloud.size(); i++)
// 	{
// 		point_cloud_sum.x = point_cloud_sum.x + point_cloud.at(i).x;
// 		point_cloud_sum.y = point_cloud_sum.y + point_cloud.at(i).y;
// 		point_cloud_sum.z = point_cloud_sum.z + point_cloud.at(i).z;
// 	}

// 	point_cloud_ave.x = point_cloud_sum.x / point_cloud.size();
// 	point_cloud_ave.y = point_cloud_sum.y / point_cloud.size();
// 	point_cloud_ave.z = point_cloud_sum.z / point_cloud.size();

// 	MatrixXf m(3, point_cloud.size());

// 	for(int i=0; i<point_cloud.size(); i++){
// 		m(0, i) = point_cloud.at(i).x - point_cloud_ave.x;
// 		m(1, i) = point_cloud.at(i).y - point_cloud_ave.y;
// 		m(2, i) = point_cloud.at(i).z - point_cloud_ave.z;
// 	}

// 	MatrixXf m2(point_cloud.size(),3);
// 	m2 = m.transpose();

// 	Matrix3f m3;
// 	m3 = (m * m2) / point_cloud.size();

// 	if(!isnan(m3(0))){
// 		// cout << "Here is the matrix m3:\n" << m3 << endl;
// 		SelfAdjointEigenSolver<Matrix3f> eigensolver(m3);
// 		if (eigensolver.info() != Success) abort();

// 		Vector3f eigenvalues;
// 		eigenvalues = eigensolver.eigenvalues();
// 		Matrix3f eigenvectors;
// 		eigenvectors = eigensolver.eigenvectors();

// 	 	if(eigenvalues(0)>eigenvalues(1) && eigenvalues(0)>eigenvalues(2)){
// 		 	eigenvector(0) = eigenvectors(0);
// 		 	eigenvector(1) = eigenvectors(1);
// 		 	eigenvector(2) = eigenvectors(2);
// 	 	}
// 	 	else if(eigenvalues(1)>eigenvalues(0) && eigenvalues(1)>eigenvalues(2)){
// 		 	eigenvector(0) = eigenvectors(3);
// 		 	eigenvector(1) = eigenvectors(4);
// 		 	eigenvector(2) = eigenvectors(5);
// 	 	}
// 	 	else if(eigenvalues(2)>eigenvalues(0) && eigenvalues(2)>eigenvalues(1)){
// 		 	eigenvector(0) = eigenvectors(6);
// 		 	eigenvector(1) = eigenvectors(7);
// 		 	eigenvector(2) = eigenvectors(8);
// 	 	}
// 	}
// }

void Object_callback::boundingbox_callback(const std::shared_ptr<darknet_ros_msgs::msg::BoundingBoxes> boxes)
{

  // RCLCPP_INFO(get_logger(), "boundingbox_callback");

  objects.detections.clear();
  
  
  visualization_msgs::msg::MarkerArray marker_array;
	
  
  target_label_1 = "bottle";

	for(int i=0; i< (int)boxes->bounding_boxes.size(); i++)
  {
    if(boxes->bounding_boxes[i].class_id == target_label_1)
    {

      geometry_msgs::msg::Point object_sampling_position;
      geometry_msgs::msg::Point pixel_top_left, pixel_bottom_right, pixel_center;
      // RCLCPP_INFO(get_logger(), "bottle");


      pixel_top_left.x = boxes->bounding_boxes[i].xmin;
      pixel_top_left.y = boxes->bounding_boxes[i].ymin;
      pixel_bottom_right.x = boxes->bounding_boxes[i].xmax;
      pixel_bottom_right.y = boxes->bounding_boxes[i].ymax;
      pixel_center.x = (boxes->bounding_boxes[i].xmin + boxes->bounding_boxes[i].xmax)/2;
      pixel_center.y = (boxes->bounding_boxes[i].ymin + boxes->bounding_boxes[i].ymax)/2;

      // if(boxes->bounding_boxes[i].class_id.compare(target_label_1)==0){
      //   object.label = target_label_1;
      // }
      // else if(boxes->bounding_boxes[i].class_id.compare(target_label_2)==0){
      //   object.label = target_label_2;
      // }
      int count = 0;
      if(depth_is_subscribed_){
        count = 0;
        // object_sampling_position = Eigen::Vector3f::Zero();

        //複数の点のpointcloudの平均を取る
        for(int t=0; t<sqrt(sampling_num); t++){
          for(int s=0; s<sqrt(sampling_num); s++){
            if(0<(pixel_center.x + sampling_pixel_step*(t-1))
               && (pixel_center.x + sampling_pixel_step*(t-1))<cols_
               && 0<(pixel_center.y + sampling_pixel_step*(s-1))
               && (pixel_center.y + sampling_pixel_step*(s-1))<rows_){

              if(!isnan(depth.at(pixel_center.x + sampling_pixel_step*(t-1), pixel_center.y + sampling_pixel_step*(s-1)).z)){
                pcl::PointXYZ point = depth.at(pixel_center.x + sampling_pixel_step*(t-1), pixel_center.y + sampling_pixel_step*(s-1));
                object_sampling_position.x += point.y;//カメラ座標系x軸 //入れ替え x,y inoue
                object_sampling_position.y -= point.x;//カメラ座標系x軸
                object_sampling_position.z += point.z;//カメラ座標系x軸
                count++;
              }
              // RCLCPP_INFO(get_logger(), "bbbbbbb");
            }
          }
        }

        
        
        RCLCPP_INFO(get_logger(), "x=%lf,y=%lf,z=%lf", object_sampling_position.x,object_sampling_position.y,object_sampling_position.z);

        if(count==0)
        {
          RCLCPP_INFO(get_logger(), "count==0");
          continue; //  0で割るとnanになるので例外処理。0になる場合が割とある
        }
        
        vision_msgs::msg::Detection3D object;
        object.header.frame_id = "camera_link";
        object.id = boxes->bounding_boxes[i].class_id; 
        object.bbox.center.position.z = -object_sampling_position.x/count;//カメラ座標系x軸
        object.bbox.center.position.y = object_sampling_position.y/count;//カメラ座標系y軸
        object.bbox.center.position.x = object_sampling_position.z/count;//カメラ座標系z軸
        object.bbox.center.orientation.w = 1; //  仮 


        vision_msgs::msg::ObjectHypothesisWithPose obj_pose;  //候補
        obj_pose.hypothesis.class_id = boxes->bounding_boxes[i].class_id; 
        obj_pose.hypothesis.score = boxes->bounding_boxes[i].probability;
        obj_pose.pose.pose = object.bbox.center;

        if(isnan(object.bbox.center.position.x))
        {
          RCLCPP_ERROR(get_logger(), "error");
          continue;
        }

        object.results.push_back(obj_pose); //候補を追加
        

        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "camera_link";
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.ns = "bottle";
        marker.id = 0;
        marker.pose = object.bbox.center;
        marker.type = visualization_msgs::msg::Marker::CYLINDER;
        marker.scale.x = 0.07;  //直径
        marker.scale.y = 0.07;  //直径
        marker.scale.z = 0.21;  //高さ
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0f;
        marker_array.markers.push_back(marker);
        // float x,y,z;
        // x= object_sampling_position(0)/count;//カメラ座標系x軸
        // y=object_sampling_position(1)/count;//カメラ座標系y軸
        // z = object_sampling_position(2)/count;//カメラ座標系z軸
        // RCLCPP_INFO(get_logger(), "x=%lf, y=%lf,z=%lf",x,y,z);

        // object.from_camera.pose.position.x = object_sampling_position(0)/count;//カメラ座標系x軸
        // object.from_camera.pose.position.y = object_sampling_position(1)/count;//カメラ座標系y軸
        // object.from_camera.pose.position.z = object_sampling_position(2)/count;//カメラ座標系z軸

        // if(object.from_camera.pose.position.z>0 && object.from_camera.pose.position.z<3.0){
        //   execute_eigenvector(pixel_top_left, pixel_bottom_right, depth, object.from_camera.pose.position.z);
        //   // cout << "eigenvector(" << i <<") =\n" << eigenvector << endl;

        //   object.from_camera.eigenvector.x = eigenvector(0);
        //   object.from_camera.eigenvector.y = eigenvector(1);
        //   object.from_camera.eigenvector.z = eigenvector(2);

        //   Quaternionf quaternion = Quaternionf::FromTwoVectors(eigenvector, Vector3f::UnitZ());

        //   object.from_camera.pose.orientation.x = quaternion.x();
        //   object.from_camera.pose.orientation.y = quaternion.y();
        //   object.from_camera.pose.orientation.z = quaternion.z();
        //   object.from_camera.pose.orientation.w = quaternion.w();

        //   msg.objects.push_back(object);
        // }
        objects.detections.push_back(object);
      }

    }
	}
  pub_object_pose_->publish(objects);
  pub_marksers_->publish(marker_array);
	// // std::cout << "boxes->bounding_boxes.size() = " << boxes->bounding_boxes.size() << std::endl;
	// // std::cout << "marker_results_yolo_msg.markers.size() = " << marker_results_yolo_msg.markers.size() << std::endl;
}



void Object_callback::cloud_callback(const std::shared_ptr<sensor_msgs::msg::PointCloud2> input) 
{
  // RCLCPP_INFO(get_logger(), "cloud_callback");
	rows_ = input->height;
  cols_ = input->width;
  pcl::fromROSMsg( *input, depth);
  depth_is_subscribed_ = true;
}

void Object_callback::show_image_callback(const std::shared_ptr<sensor_msgs::msg::Image> image)
{
  // RCLCPP_INFO(get_logger(), "Show_image_with_Callback");
  cv_bridge::CvImagePtr cv_ptr;

  try{
      cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
  }catch (cv_bridge::Exception& ex){
      RCLCPP_ERROR(get_logger(), "error");
      exit(-1);
  }

  cv::Mat Image_with_labels(cv_ptr->image.rows, cv_ptr->image.cols, CV_8UC3);

  Image_with_labels = cv_ptr->image;

	cols_ = cv_ptr->image.cols;
	rows_ = cv_ptr->image.rows;
  // RCLCPP_INFO(get_logger(), "cols=%d,rows=%d",cols, rows);
	// if(show_image_window_object_callback){
	// 	// ROS_INFO_STREAM("show_image_window_object_callback");
	// 	for(int i=0; i<msg.objects.size(); i++){

  //     cv::rectangle(Image_with_labels, cv::Point(msg.objects[i].pixel_center.x-sampling_pixel_step, msg.objects[i].pixel_center.y-sampling_pixel_step), cv::Point(msg.objects[i].pixel_center.x+sampling_pixel_step, msg.objects[i].pixel_center.y+sampling_pixel_step), cv::Scalar(255, 255, 255), 3, 4);

  //     double putText_x;

  //     if(msg.objects[i].pixel_center.x>cols-300){
  //       putText_x = cols-330;
  //     }
  //     else{
  //       putText_x = msg.objects[i].pixel_center.x-10;
  //     }

  //     char z_1[1000];
  //     sprintf(z_1, "label: %s", msg.objects[i].label.c_str());
  //     cv::putText(Image_with_labels, z_1, cv::Point(putText_x, msg.objects[i].pixel_center.y-60), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2, 2, false);

  //     char z_2[1000];
  //     sprintf(z_2, "pixel: (%lu, %lu)", msg.objects[i].pixel_center.x, msg.objects[i].pixel_center.y);
  //     cv::putText(Image_with_labels, z_2, cv::Point(putText_x, msg.objects[i].pixel_center.y-42), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 2, 2, false);

  //     char z_3[1000];
  //     sprintf(z_3, "position: (%4.4f, %4.4f, %4.4f)", msg.objects[i].from_camera.pose.position.x, msg.objects[i].from_camera.pose.position.y, msg.objects[i].from_camera.pose.position.z);
  //     cv::putText(Image_with_labels, z_3, cv::Point(putText_x, msg.objects[i].pixel_center.y-25), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 2, 2, false);

  //     char z_4[1000];
  //     sprintf(z_4, "eigenvector: (%4.4f, %4.4f, %4.4f)", msg.objects[i].from_camera.eigenvector.x, msg.objects[i].from_camera.eigenvector.y, msg.objects[i].from_camera.eigenvector.z);
  //     cv::putText(Image_with_labels, z_4, cv::Point(putText_x, msg.objects[i].pixel_center.y-8), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 2, 2, false);
	// 	}

	//   cv::imshow("Image_with_labels", Image_with_labels);
	//   cv::waitKey(10);
	// }

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

	// while (ros::ok()){
  //   ros::spinOnce();
  //   loop_rate.sleep();
	// 	object_callback.pub_object_pose.publish(object_callback.msg);
  // }

  return 0;
}
