// Copyright 2021 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <rclcpp/rclcpp.hpp>
#include "rclcpp/logger.hpp"
#include <darknet_ros_msgs/msg/bounding_boxes.hpp>
#include <darknet_ros_msgs/msg/bounding_box.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>

#include <XnOpenNI.h>
#include <XnCodecIDs.h>
#include <XnCppWrapper.h>
#include <XnPropNames.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <behavior_tree_msgs/msg/pixel.hpp>
#include <behavior_tree_msgs/msg/object.hpp>
#include <behavior_tree_msgs/msg/object_array.hpp>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/LU>

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>

#include <memory>
#include <string>

// int cols, rows;
// std::string depth_frame;
// sensor_msgs::PointCloud2 point_cloud_;
// bool depth_is_subscribed;

class ObjectPub : public rclcpp::Node
{ 
public:
  ObjectPub()
  : Node("objectpub"),tf_buffer_(this->get_clock()),  // tf_buffer_ を正しく初期化
      tf_listener_(tf_buffer_)
  {
    rclcpp::QoS qos(rclcpp::KeepLast(10));
    qos.reliable();  // ★
    qos.durability_volatile();
    
    box_sub_ = this->create_subscription<darknet_ros_msgs::msg::BoundingBoxes>(
            "/darknet_ros/bounding_boxes", qos, std::bind(&ObjectPub::boundingbox_callback, this, std::placeholders::_1));
    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/amir1/camera/depth/color/points",qos, std::bind(&ObjectPub::cloud_callback, this, std::placeholders::_1));
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/amir1/camera/color/image_raw",qos, std::bind(&ObjectPub::image_callback, this, std::placeholders::_1));
    depth_is_subscribed = false;
    sampling_pixel_step = 3;
    sampling_num = pow(3.0, 2.0);//
    timer_ = this->create_wall_timer(std::chrono::milliseconds(33), std::bind(&ObjectPub::timerCallback, this));

    pub_object_pose_ = this->create_publisher<behavior_tree_msgs::msg::ObjectArray>("/amir1/object_position", 10); //カメラ座標系のオブジェクトの位置

  }
  void  timerCallback()
  {
    if (!depth_is_subscribed) {
        return;
    }

    try {
        // "map" フレームから "depth_frame" への変換を取得
        transformStamped_to_map_ = tf_buffer_.lookupTransform("map", depth_frame, tf2::TimePointZero);
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN(this->get_logger(), "%s", ex.what());
    }

    try {
        // "base_link" フレームから "depth_frame" への変換を取得
        transformStamped_to_base_link_ = tf_buffer_.lookupTransform("base_link", depth_frame, tf2::TimePointZero);
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN(this->get_logger(), "%s", ex.what());
    }

  }

  Eigen::Vector3d execute_eigenvector(const behavior_tree_msgs::msg::Pixel& pixel_top_left, 
      const behavior_tree_msgs::msg::Pixel& pixel_bottom_right, 
      const pcl::PointCloud<pcl::PointXYZ>& depth, 
      const geometry_msgs::msg::Point& center_point
    )
  {
    //
    Eigen::Vector3d eigenvector;
    std::vector<pcl::PointXYZ> point_cloud;
    pcl::PointXYZ point;

    geometry_msgs::msg::Point center_point_in_base;

    for(int i=pixel_top_left.x; i<pixel_bottom_right.x; i++){
      for(int j=pixel_top_left.y; j<pixel_bottom_right.y; j++){
        geometry_msgs::msg::Point p_in_camera, p_in_base, center_point_in_base;
        p_in_camera.x = point.x;
        p_in_camera.y = point.y;
        p_in_camera.z = point.z;
        
        // tf2::doTransform(p_in_camera, p_in_base, transformStamped_to_arm_2_link_1_);
        // if (p_in_base.z > -0.152 && p_in_base.x > center_point_in_base.x - 0.1)

        if(depth.at(i,j).z < center_point.z + 0.1)
        {
          point.x = depth.at(i,j).x;
          point.y = depth.at(i,j).y;
          point.z = depth.at(i,j).z;
          point_cloud.push_back(point);
        }
      }
    }
    pcl::PointXYZ point_cloud_sum, point_cloud_ave;

    //  平均位置ベクトルを計算
    for(int i=0; i<point_cloud.size(); i++)
    {
      point_cloud_sum.x = point_cloud_sum.x + point_cloud.at(i).x;
      point_cloud_sum.y = point_cloud_sum.y + point_cloud.at(i).y;
      point_cloud_sum.z = point_cloud_sum.z + point_cloud.at(i).z;
    }

    point_cloud_ave.x = point_cloud_sum.x / point_cloud.size();
    point_cloud_ave.y = point_cloud_sum.y / point_cloud.size();
    point_cloud_ave.z = point_cloud_sum.z / point_cloud.size();

    //  共分散行列を計算
    Eigen::MatrixXd m(3, point_cloud.size());

    for(int i=0; i<point_cloud.size(); i++){
      m(0, i) = point_cloud.at(i).x - point_cloud_ave.x;
      m(1, i) = point_cloud.at(i).y - point_cloud_ave.y;
      m(2, i) = point_cloud.at(i).z - point_cloud_ave.z;
    }

    Eigen::Matrix3d m3;
    m3 = (m * m.transpose()) / point_cloud.size();
    
    //  上記の共分散行列の固有値と固有ベクトルを計算。最大の固有値に対応する固有ベクトルが主軸ベクトル
    // if(!std::isnan(m3(0))){
    //   // cout << "Here is the matrix m3:\n" << m3 << endl;
    //   Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver(m3); //  固有値と固有ベクトルを計算
    //   if (eigensolver.info() != Eigen::Success) abort();

    //   Eigen::Vector3d eigenvalues;
    //   eigenvalues = eigensolver.eigenvalues();
    //   Eigen::Matrix3d eigenvectors;
    //   eigenvectors = eigensolver.eigenvectors();

    //   if(eigenvalues(0)>eigenvalues(1) && eigenvalues(0)>eigenvalues(2)){ //  固有値(0)が最大の場合
    //     eigenvector(0) = eigenvectors(0);
    //     eigenvector(1) = eigenvectors(1);
    //     eigenvector(2) = eigenvectors(2);
    //   }
    //   else if(eigenvalues(1)>eigenvalues(0) && eigenvalues(1)>eigenvalues(2)){ //  固有値(1)が最大の場合
    //     eigenvector(0) = eigenvectors(3);
    //     eigenvector(1) = eigenvectors(4);
    //     eigenvector(2) = eigenvectors(5);
    //   }
    //   else if(eigenvalues(2)>eigenvalues(0) && eigenvalues(2)>eigenvalues(1)){ //  固有値(2)が最大の場合
    //     eigenvector(0) = eigenvectors(6);
    //     eigenvector(1) = eigenvectors(7);
    //     eigenvector(2) = eigenvectors(8);
    //   }
    // }

    if (!std::isnan(m3(0, 0))) {
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver(m3);
      if (eigensolver.info() == Eigen::Success) {
        int max_index;
        eigensolver.eigenvalues().maxCoeff(&max_index);
        eigenvector = eigensolver.eigenvectors().col(max_index);
      } else {
        RCLCPP_WARN(this->get_logger(), "Eigen decomposition failed.");
      }
    } else {
      RCLCPP_WARN(this->get_logger(), "Covariance matrix contains NaN.");
    }
    return eigenvector;

  }

  void boundingbox_callback(const darknet_ros_msgs::msg::BoundingBoxes::SharedPtr boxes)
  { 
    if (!depth_is_subscribed) return;
    pcl::fromROSMsg(point_cloud_, depth);

    //  コールバック内でメッセージを生成して最後に保存する(直接書き換えると処理中の状態でpublishしてしまう)
    std::vector<behavior_tree_msgs::msg::Object> detected_objects;
    // std::vector<behavior_tree_msgs::Object> bb_objects;

    int count;
    Eigen::Vector3d object_sampling_position;

    for (const auto& bounding_box: boxes->bounding_boxes)
    {
        // if (bounding_box.class_id != "bottle") {
        //     continue;  // ボトル以外のオブジェクトをスキップ
        // }

        if (bounding_box.class_id != "bottle") {
            continue;  // お茶の箱以外のオブジェクトをスキップ
        }

        behavior_tree_msgs::msg::Object object;
        object.pixel_top_left.x = bounding_box.xmin;
        object.pixel_top_left.y = bounding_box.ymin;
        object.pixel_bottom_right.x = bounding_box.xmax;
        object.pixel_bottom_right.y = bounding_box.ymax;
        object.pixel_center.x = (bounding_box.xmin + bounding_box.xmax)/2;
        object.pixel_center.y = (bounding_box.ymin + bounding_box.ymax)/2;
        object.label = bounding_box.class_id;
        object.probability = bounding_box.probability;

        count = 0;
        object_sampling_position = Eigen::Vector3d::Zero();

        for(int t=0; t<sqrt(sampling_num); t++){
          for(int s=0; s<sqrt(sampling_num); s++){
            if(0<(object.pixel_center.x + sampling_pixel_step*(t-1))
              && (object.pixel_center.x + sampling_pixel_step*(t-1))<cols
              && 0<(object.pixel_center.y + sampling_pixel_step*(s-1))
              && (object.pixel_center.y + sampling_pixel_step*(s-1))<rows){

              pcl::PointXYZ point = depth.at(object.pixel_center.x + sampling_pixel_step*(t-1), object.pixel_center.y + sampling_pixel_step*(s-1));

              if(!std::isnan(point.x) && !std::isnan(point.y) && !std::isnan(point.z)){
                object_sampling_position(0) += point.x; //カメラ座標系x軸 //入れ替え x,y inoue
                object_sampling_position(1) += point.y; //カメラ座標系x軸
                object_sampling_position(2) += point.z; //カメラ座標系x軸
                count++;
              }
            }
          }
        }
        
      if(count==0)
      {
        // ROS_INFO("object_callback: count==0");
        continue; //  0で割るとnanになるので例外処理。rgbカメラで画像認識しても、depthカメラに写ってない場合がある。オクルージョンが原因？
      }
      object.from_camera.position.x = object_sampling_position(0)/count;//カメラ座標系x軸
      object.from_camera.position.y = object_sampling_position(1)/count;//カメラ座標系y軸
      object.from_camera.position.z = object_sampling_position(2)/count;//カメラ座標系z軸

      Eigen::Vector3d eigenvector;
      if (object.label == "bottle")
      {
        double width = bounding_box.xmax - bounding_box.xmin;
        double height = bounding_box.ymax - bounding_box.ymin;
        // ROS_INFO("width=%lf,height=%lf", width, height);
        // ROS_INFO("ratio=%lf", height/width);

        eigenvector = execute_eigenvector(object.pixel_top_left, object.pixel_bottom_right, depth, object.from_camera.position);

        Eigen::Quaterniond quaternion = Eigen::Quaterniond::FromTwoVectors(eigenvector, Eigen::Vector3d::UnitZ());
        object.from_camera.orientation.x = quaternion.x();
        object.from_camera.orientation.y = quaternion.y();
        object.from_camera.orientation.z = quaternion.z();
        object.from_camera.orientation.w = quaternion.w();
      }
      else
      {
        object.from_camera.orientation.w = 1;
      }

      tf2::doTransform(object.from_camera, object.from_odom, transformStamped_to_map_);
      tf2::doTransform(object.from_camera, object.from_base, transformStamped_to_base_link_);

      detected_objects.push_back(object);

    }
    msg.objects = std::move(detected_objects); //  コピー中にpublishすると問題が発生するのでstd::moveを使う

    // pub_object_pose_->publish(msg); //カメラ座標系のオブジェクトの位置をpublish
  } 

  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud)
  {
    depth_frame =cloud->header.frame_id;
    point_cloud_ = *cloud;
    // pcl::fromROSMsg(point_cloud_, depth);
    // ROS_INFO("height=%d, width=%d",input->height, input->width);
    cols = cloud->width;
    rows = cloud->height;
    depth_is_subscribed = true;
  }

  void image_callback(const sensor_msgs::msg::Image::SharedPtr image)
  {
    //
    cv_bridge::CvImagePtr cv_ptr;

    try {
        cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& ex) {
        RCLCPP_ERROR(this->get_logger(), "CV Bridge error: %s", ex.what());
        return; 
    }

    // cv::Mat Image_with_labels = cv_ptr->image.clone();
    cv::Mat Image_with_labels(cv_ptr->image.rows, cv_ptr->image.cols, CV_8UC3);
    Image_with_labels = cv_ptr->image;
    cols = cv_ptr->image.cols;
    rows = cv_ptr->image.rows;

    for(int i=0; i<msg.objects.size(); i++){

      cv::rectangle(Image_with_labels, cv::Point(msg.objects[i].pixel_center.x-sampling_pixel_step, msg.objects[i].pixel_center.y-sampling_pixel_step), cv::Point(msg.objects[i].pixel_center.x+sampling_pixel_step, msg.objects[i].pixel_center.y+sampling_pixel_step), cv::Scalar(255, 255, 255), 3, 4);

      double putText_x;

      if(msg.objects[i].pixel_center.x>cols-300){
        putText_x = cols-330;
      }
      else{
        putText_x = msg.objects[i].pixel_center.x-10;
      }

      char z_1[1000];
      sprintf(z_1, "label: %s", msg.objects[i].label.c_str());
      cv::putText(Image_with_labels, z_1, cv::Point(putText_x, msg.objects[i].pixel_center.y-60), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2, 2, false);

      char z_2[1000];
      sprintf(z_2, "pixel: (%lu, %lu)", msg.objects[i].pixel_center.x, msg.objects[i].pixel_center.y);
      cv::putText(Image_with_labels, z_2, cv::Point(putText_x, msg.objects[i].pixel_center.y-42), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 2, 2, false);

      char z_3[1000];
      sprintf(z_3, "position: (%4.4f, %4.4f, %4.4f)", msg.objects[i].from_camera.position.x, msg.objects[i].from_camera.position.y, msg.objects[i].from_camera.position.z);
      cv::putText(Image_with_labels, z_3, cv::Point(putText_x, msg.objects[i].pixel_center.y-25), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 2, 2, false);
		}

	  cv::imshow("Image_with_labels", Image_with_labels);
	  cv::waitKey(10);

  }
  rclcpp::Subscription<darknet_ros_msgs::msg::BoundingBoxes>::SharedPtr box_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  int cols, rows;
  std::string depth_frame;
  sensor_msgs::msg::PointCloud2 point_cloud_;
  bool depth_is_subscribed;
  pcl::PointCloud<pcl::PointXYZ> depth;
  behavior_tree_msgs::msg::ObjectArray msg;
  geometry_msgs::msg::TransformStamped transformStamped_to_map_, transformStamped_to_base_link_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  // int sampling_pixel_step = 3;
	// int sampling_num = pow(3.0, 2.0);//n^2をいれること
  int sampling_pixel_step;
  int sampling_num;
  //int sampling_num = 3 * 3;
  rclcpp::Publisher<behavior_tree_msgs::msg::ObjectArray>::SharedPtr pub_object_pose_;

private:
  // rclcpp::Subscription<darknet_ros_msgs::msg::BoundingBoxes>::SharedPtr box_sub_;
  // rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  // rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  // int cols, rows;
  // std::string depth_frame;
  // sensor_msgs::msg::PointCloud2 point_cloud_;
  // bool depth_is_subscribed;
  // pcl::PointCloud<pcl::PointXYZ> depth;
  // behavior_tree_msgs::msg::ObjectArray msg;
  // geometry_msgs::msg::TransformStamped transformStamped_to_map_, transformStamped_to_base_link_;
  // tf2_ros::Buffer tf_buffer_;
  // tf2_ros::TransformListener tf_listener_;
  // // int sampling_pixel_step = 3;
	// // int sampling_num = pow(3.0, 2.0);//n^2をいれること
  // int sampling_pixel_step;
  // int sampling_num;
  // //int sampling_num = 3 * 3;
  // rclcpp::Publisher<behavior_tree_msgs::msg::ObjectArray>::SharedPtr pub_object_pose_;
};
  

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  // rclcpp::spin(std::make_shared<ObjectPub>());
  auto node = std::make_shared<ObjectPub>();
  rclcpp::Rate loop_rate(30);
  while (rclcpp::ok()) {
      // オブジェクト位置とオブジェクト配列をパブリッシュ
      node->pub_object_pose_->publish(node->msg);
      // node->pub_object_array_->publish(node->object_array);

      // ノードのコールバック処理を実行
      // rclcpp::spin_some(node);
      rclcpp::spin_some(node);
      loop_rate.sleep();
    }
  rclcpp::shutdown();
  return 0;
}

