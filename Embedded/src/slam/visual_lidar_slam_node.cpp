#include "amr/slam/visual_lidar_slam_node.h"

#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "opencv2/features2d.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace amr {

VisualLidarSlamNode::VisualLidarSlamNode()
: Node("visual_lidar_slam_node")
{
  camera_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "/camera/image_raw", 10,
    std::bind(&VisualLidarSlamNode::cameraCallback, this, std::placeholders::_1));

  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan", 10,
    std::bind(&VisualLidarSlamNode::lidarCallback, this, std::placeholders::_1));

  cv::namedWindow("Visual SLAM Matches", cv::WINDOW_NORMAL);
}

// OpenCV 상태 변수
cv::Mat VisualLidarSlamNode::prev_image_;
std::vector<cv::KeyPoint> VisualLidarSlamNode::prev_keypoints_;
cv::Mat VisualLidarSlamNode::prev_descriptors_;

void VisualLidarSlamNode::cameraCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
  if (!msg || msg->data.empty()) {
    RCLCPP_WARN(this->get_logger(), "Received empty camera image");
    return;
  }

  // ROS → OpenCV 이미지 변환
  cv::Mat image;
  try {
    image = cv_bridge::toCvCopy(msg, "bgr8")->image;
  } catch (cv_bridge::Exception& e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  // 특징점 검출 및 디스크립터 생성
  cv::Ptr<cv::ORB> orb = cv::ORB::create();
  std::vector<cv::KeyPoint> keypoints;
  cv::Mat descriptors;
  orb->detectAndCompute(image, cv::noArray(), keypoints, descriptors);

  // 이전 프레임이 있을 경우 매칭
  if (!prev_image_.empty()) {
    cv::BFMatcher matcher(cv::NORM_HAMMING, true);
    std::vector<cv::DMatch> matches;
    matcher.match(prev_descriptors_, descriptors, matches);

    // 매칭 시각화
    cv::Mat match_img;
    cv::drawMatches(prev_image_, prev_keypoints_, image, keypoints, matches, match_img);
    cv::imshow("Visual SLAM Matches", match_img);
    cv::waitKey(1);
  }

  // 상태 갱신
  prev_image_ = image.clone();
  prev_keypoints_ = keypoints;
  prev_descriptors_ = descriptors.clone();
}

void VisualLidarSlamNode::lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  if (!msg || msg->ranges.empty()) {
    RCLCPP_WARN(this->get_logger(), "Received empty LiDAR scan");
    return;
  }

  float angle = msg->angle_min;
  std::vector<cv::Point2f> scan_points;

  for (const auto& range : msg->ranges) {
    if (std::isfinite(range)) {
      float x = range * std::cos(angle);
      float y = range * std::sin(angle);
      scan_points.emplace_back(x, y);
    }
    angle += msg->angle_increment;
  }

  RCLCPP_INFO_ONCE(this->get_logger(), "LiDAR scan processed. First scan received.");
}

}  
