#ifndef AMR_VISUAL_LIDAR_SLAM_NODE_H
#define AMR_VISUAL_LIDAR_SLAM_NODE_H

#include "rclcpp/rclcpp.h"
#include <sensor_msgs/msg/image.h>
#include <sensor_msgs/msg/laser_scan.h>
#include <opencv2/core.h>
#include <opencv2/features2d.h>

namespace amr {

class VisualLidarSlamNode : public rclcpp::Node {
public:
  VisualLidarSlamNode();

private:
  void cameraCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  void lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

  // ROS2 Subscribers
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;

  // OpenCV 상태 변수
  static cv::Mat prev_image_;
  static std::vector<cv::KeyPoint> prev_keypoints_;
  static cv::Mat prev_descriptors_;
};

}  

#endif
