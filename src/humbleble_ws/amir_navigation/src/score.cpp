#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <darknet_ros_msgs/msg/bounding_boxes.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <iomanip>
#include <cmath>
#include <sstream>
#include <algorithm>
#include "geometry_msgs/msg/pose_stamped.hpp"

using std::placeholders::_1;

class ScoredBottleSelector : public rclcpp::Node
{
public:
    ScoredBottleSelector() : Node("scored_bottle_selector")
    {
        // ボトルの3D座標を送信するPublisher（最も良い候補1つのみ）
        bottle_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/score_position", 10);

        // アノテーション付き画像を送信するPublisher（すべての候補）
        overlay_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/image_overlay", 10);

        // カメラインフォ（内部パラメータ）受信
        cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/amir1/camera/aligned_depth_to_color/camera_info", 10,
            std::bind(&ScoredBottleSelector::cameraInfoCallback, this, _1));

        // 深度画像受信
        depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/amir1/camera/aligned_depth_to_color/image_raw", 10,
            std::bind(&ScoredBottleSelector::depthCallback, this, _1));

        // RGB画像受信
        color_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/amir1/camera/color/image_raw",
            rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(),
            std::bind(&ScoredBottleSelector::colorCallback, this, _1));

        // YOLOのBounding Box受信
        bbox_sub_ = this->create_subscription<darknet_ros_msgs::msg::BoundingBoxes>(
            "/darknet_ros/bounding_boxes", 10,
            std::bind(&ScoredBottleSelector::bboxCallback, this, _1));
    }

private:
    // 保持用メンバ変数
    sensor_msgs::msg::CameraInfo::SharedPtr camera_info_; // カメラ内部パラメータ
    cv::Mat depth_image_;                                 // 最新の深度画像
    cv::Mat color_image_;                                 // 最新のRGB画像

    // Publisher/Subscriber
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr bottle_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr overlay_pub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr color_sub_;
    rclcpp::Subscription<darknet_ros_msgs::msg::BoundingBoxes>::SharedPtr bbox_sub_;

    // カメラインフォ受信時の処理
    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        camera_info_ = msg;
    }

    // 深度画像受信時の処理
    void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        depth_image_ = cv_bridge::toCvCopy(msg, msg->encoding)->image;
    }

    // RGB画像受信時の処理
    void colorCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        color_image_ = cv_bridge::toCvCopy(msg, "bgr8")->image;
    }

    // バウンディングボックス受信時の処理（メイン処理）
    void bboxCallback(const darknet_ros_msgs::msg::BoundingBoxes::SharedPtr msg)
    {
        // すべての情報が揃っているか確認
        if (!camera_info_ || depth_image_.empty() || color_image_.empty()) {
            RCLCPP_INFO(this->get_logger(), "データ準備中（カメラインフォ、深度、カラー画像）...");
            return;
        }

        // 画像中心と最大距離（正規化用）
        int img_center_x = color_image_.cols / 2;
        int img_center_y = color_image_.rows / 2;
        double max_dist = std::sqrt(img_center_x * img_center_x + img_center_y * img_center_y);

        // スコア最大のボトル情報を保存
        double best_score = -1.0;
        geometry_msgs::msg::PoseStamped best_point;

        // 描画用画像のコピー
        cv::Mat img = color_image_.clone();

        // 各バウンディングボックスに対して処理
        for (const auto & box : msg->bounding_boxes) {
            if (box.class_id != "bottle") continue;

            // バウンディングボックスの中心画素座標
            int u = (box.xmin + box.xmax) / 2;
            int v = (box.ymin + box.ymax) / 2;

            // 深度取得（単位: mm → mに変換）
            float depth = depth_image_.at<uint16_t>(v, u);
            if (depth == 0) continue; // 無効な深度はスキップ

            float fx = camera_info_->k[0];
            float fy = camera_info_->k[4];
            float cx = camera_info_->k[2];
            float cy = camera_info_->k[5];

            float Z = depth / 1000.0;
            float X = (u - cx) * Z / fx;
            float Y = (v - cy) * Z / fy;

            // --- スコア計算 ---

            // 1. 中心との距離スコア（中心に近いほど良い）
            double center_dist = std::sqrt(std::pow(u - img_center_x, 2) + std::pow(v - img_center_y, 2));
            double center_score = 1.0 - (center_dist / max_dist); // 正規化

            // 2. 深度スコア（近すぎず遠すぎずが良い）
            const double min_depth = 0.2, max_depth = 1.0; // 0.5mより近いなら1.0、2.0mより遠いと0.0
            double depth_score = 0.0;
            if (Z <= min_depth) depth_score = 1.0;
            else if (Z >= max_depth) depth_score = 0.0;
            else depth_score = 1.0 - (Z - min_depth) / (max_depth - min_depth);

            // 3. 面積スコア（大きすぎず小さすぎず）
            double area = (box.xmax - box.xmin) * (box.ymax - box.ymin);
            const double min_area = 1000.0, max_area = 10000.0; //2000より小さいと score 0.0、20000より大きいと score 1.0
            double area_score = std::clamp((area - min_area) / (max_area - min_area), 0.0, 1.0);

            // 総合スコア（重み付き合計）
            double total_score = 0.3 * center_score + 0.5 * depth_score + 0.2 * area_score;

            // --- 画像への描画 ---
            // バウンディングボックスを描く（緑）
            cv::rectangle(img, cv::Point(box.xmin, box.ymin), cv::Point(box.xmax, box.ymax), cv::Scalar(0, 255, 0), 2);

            // ラベル（スコア付き）を描く（青文字）
            std::ostringstream label;
            label << box.class_id << " (score: " << std::fixed << std::setprecision(2) << total_score << ")";
            int text_y = std::min(static_cast<int>(box.ymax + 15), img.rows - 10); // はみ出さないよう調整
            cv::putText(img, label.str(), cv::Point(box.xmin, text_y), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 1);

            // 最良のボトルを保存
            if (total_score > best_score) {
                best_score = total_score;
                best_point.header.stamp = this->get_clock()->now();
                best_point.header.frame_id = camera_info_->header.frame_id;
                best_point.pose.position.x = X;
                best_point.pose.position.y = Y;
                best_point.pose.position.z = Z;
            }
        }

        // 最もスコアが高いボトルの3D位置をPublish
        if (best_score >= 0.0) {
            bottle_pub_->publish(best_point);
            RCLCPP_INFO(this->get_logger(), "選択されたボトル位置: [%.2f, %.2f, %.2f] (score: %.2f)",
                        best_point.pose.position.x, best_point.pose.position.y, best_point.pose.position.z, best_score);
        }

        // 描画画像をPublish
        sensor_msgs::msg::Image::SharedPtr overlay_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", img).toImageMsg();
        overlay_msg->header.stamp = this->now();
        overlay_msg->header.frame_id = camera_info_->header.frame_id;
        overlay_pub_->publish(*overlay_msg);

        // 画像表示
        cv::imshow("Annotated Image", img);
        cv::waitKey(1);

    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ScoredBottleSelector>());
    rclcpp::shutdown();
    return 0;
}