#include "amr/ros/ros_publisher.h"

RosPublisher::RosPublisher(const std::string& topic, const std::string& jsonTemplate, std::chrono::milliseconds period = std::chrono::milliseconds(500))
	: Node("ARM_PUB"), topic_(topic) {
	try {
		jsonTemplate_ = nlohmann::json::parse(jsonTemplate);
	}
	catch (const nlhomann::json::parse_error& e) {
		RCLCPP_ERROR(this->get_logger(), "Invalid Json Template : %s", e.what());
		throw;
	}
	publisher_ = this->create_publisher<std_msgs::msg::String>(topic_, 10);
	timer_ = this->create_wall_timer(period, std::bind(&timerCallback, this));
}

void RosPublisher::timerCallback() {
	//TODO : Json Field Update

	std_msgs::msg::String msg;
	msg.data = jsonTemplate_.dump();

	RCLCPP_INFO(this->get_logger(), "[%s] Publishing: %s", topic_.c_str(), msg.data.c_str());

	publisher_->publish(msg);
}