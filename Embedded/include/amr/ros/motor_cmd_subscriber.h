#pragma once
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "../motor_cmd_interface.h"

namespace amr {

class MotorCmdSubscriber : public rclcpp::Node {
public:
    explicit MotorCmdSubscriber(std::shared_ptr<MotorCmdInterface> cmdIf);

private:
    void topicCb(const std_msgs::msg::Int32::SharedPtr msg);
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_;
    std::shared_ptr<MotorCmdInterface> cmdIf_;
};

}