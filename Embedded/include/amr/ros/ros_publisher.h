#ifndef ROS_PUBLISHER_H
#define ROS_PUBLISHER_H

#include <chrono>
#include <memory>
#include <string>
#include <functional>
#include <nlohmann/json.hpp>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class RosPublisher : public rclcpp::Node{
public:
	RosPublisher(const std::string& topic, const std::string & jsonTemplate, std::chrono::milliseconds period = std::chrono::milliseconds(500));

private:
	void timerCallback();

	std::string topic_;
	nlohmann::json jsonTemplate_;
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
	rclcpp::TimerBase::SharedPtr timer_;
	size_t count_{ 0 };
};

#endif
