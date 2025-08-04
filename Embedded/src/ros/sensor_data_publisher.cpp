#include "amr/ros/sensor_data_publisher.h"

namespace amr {
namespace ros {

SensorDataPublisher::SensorDataPublisher(
    std::shared_ptr<BackendWsClient> backend,
    rclcpp::Node::SharedPtr parent_node)
    : Node("sensor_data_publisher"), backend_(backend)
{
  lidar_ = std::make_shared<amr::LidarSensor>(parent_node);
  pub_ = this->create_publisher<std_msgs::msg::Float32>("/sensor_data", 10);

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500),  // 500ms
    std::bind(&SensorDataPublisher::timerCallback, this)
  );
}

void SensorDataPublisher::timerCallback() {
  float distance = lidar_->getDistance();

  std_msgs::msg::Float32 msg;
  msg.data = distance;
  pub_->publish(msg);
  RCLCPP_INFO(this->get_logger(), "센서 거리 퍼블리시: %.2f", distance);

  if (backend_) {
    backend_->sendLog("Sensor distance: " + std::to_string(distance));
  } else {
    RCLCPP_WARN(this->get_logger(), "BackendWsClient 연결 없음");
  }
}

} 
} 
