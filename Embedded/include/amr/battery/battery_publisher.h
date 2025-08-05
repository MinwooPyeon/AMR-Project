#ifndef AMR_BATTERY_PUBLISHER_H
#define AMR_BATTERY_PUBLISHER_H

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"

namespace amr {

class BatteryPublisher : public rclcpp::Node {
public:
  BatteryPublisher();

private:
  void timerCallback();

  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}

#endif 
