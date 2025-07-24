#include "amr/battery/battery_publisher.h"

namespace amr {

BatteryPublisher::BatteryPublisher()
: Node("battery_publisher")
{
  battery_pub_ = this->create_publisher<sensor_msgs::msg::BatteryState>("/battery_state", 10);
  timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&BatteryPublisher::timerCallback, this));
}

void BatteryPublisher::timerCallback() {
  auto msg = sensor_msgs::msg::BatteryState();
  // 배터리 센서 API로 바꿔야함 (임시)
  msg.voltage = 12.4f;
  msg.current = 1.5f;
  msg.percentage = 0.75f;
  msg.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;

  if(msg.percentage < 0.2f) {
    RCLCPP_WARN(this->get_logger(), "Battery low: %.0f%% remaining", msg.percentage * 100.0);
  }
  battery_pub_->publish(msg);
}

} 
