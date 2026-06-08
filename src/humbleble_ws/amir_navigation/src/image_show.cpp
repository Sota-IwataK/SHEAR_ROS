#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <darknet_ros_msgs/msg/bounding_boxes.hpp>
#include <iomanip>
#include <cmath>
#include <sstream>

class AnnotatedImageViewer : public rclcpp::Node
{
public:
  AnnotatedImageViewer()
  : Node("annotated_image_viewer")
  {
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/amir1/annotated_image", 10,
      std::bind(&AnnotatedImageViewer::image_callback, this, std::placeholders::_1));
    bbox_sub_ = this->create_subscription<darknet_ros_msgs::msg::BoundingBoxes>(
      "/darknet_ros/bounding_boxes", 10,
      std::bind(&AnnotatedImageViewer::bbox_callback, this, std::placeholders::_1));

  }

private:
  darknet_ros_msgs::msg::BoundingBoxes::SharedPtr latest_boxes_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<darknet_ros_msgs::msg::BoundingBoxes>::SharedPtr bbox_sub_;
  void bbox_callback(const darknet_ros_msgs::msg::BoundingBoxes::SharedPtr msg)
  {
    latest_boxes_ = msg;
  }
  
  
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    try
    {
      cv::Mat img = cv_bridge::toCvCopy(msg, "bgr8")->image;
      int img_center_x = img.cols / 2;
      int img_center_y = img.rows / 2;
      double max_dist = std::sqrt(img_center_x * img_center_x + img_center_y * img_center_y);

      if (latest_boxes_)
      {
        for (const auto & box : latest_boxes_->bounding_boxes)
        {
          if (box.class_id != "bottle") continue;
          int box_center_x = (box.xmin + box.xmax) / 2;
          int box_center_y = (box.ymin + box.ymax) / 2;
          double dist = std::sqrt(std::pow(box_center_x - img_center_x, 2) + std::pow(box_center_y - img_center_y, 2));
          double score = 1.0 - (dist / max_dist);

          // 描画
          cv::rectangle(img, cv::Point(box.xmin, box.ymin), cv::Point(box.xmax, box.ymax), cv::Scalar(0, 255, 0), 2);
          std::ostringstream label;
          label << box.class_id << " (score: " << std::fixed << std::setprecision(2) << score << ")";
          // テキストが画像からはみ出さないようにY座標を調整
          int text_y = std::min(static_cast<int>(box.ymax + 15), img.rows - 10);
          // int text_y = std::max(static_cast<int>(box.ymin - 5), 10);
          cv::putText(img, label.str(), cv::Point(box.xmin, text_y), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 1);
        }
      }

      cv::imshow("Annotated Image", img);
      cv::waitKey(1);
    }
    catch (const cv_bridge::Exception & e)
    {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AnnotatedImageViewer>());
  rclcpp::shutdown();
  return 0;
}
