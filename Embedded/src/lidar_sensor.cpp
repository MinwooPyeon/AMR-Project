#include "amr/lidar_sensor.h"

#include <algorithm>
#include <numeric>
#include <cmath>


namespace amr {

LidarSensor::LidarSensor(rclcpp::Node::SharedPtr node, const std::string& topic)
: topic_(topic), distance_(0.0f)
{
  sub_ = node->create_subscription<sensor_msgs::msg::LaserScan>(
    topic_, 10,
    std::bind(&LidarSensor::laserCallback, this, std::placeholders::_1));
}


void LidarSensor::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  if (!msg || msg->ranges.empty()) return;

  // 정면 방향
  int center_idx = msg->ranges.size() / 2;
  float raw_distance = msg->ranges[center_idx];

  // 예외 처리
  if (!std::isfinite(raw_distance) || raw_distance <= 0.1f || raw_distance > 10.0f) {
    return;
  }

  // 거리 업데이트
  std::lock_guard<std::mutex> lock(mutex_);
  distance_ = raw_distance;
}


float LidarSensor::getDistance()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return distance_;
}

}
