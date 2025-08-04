#pragma once

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "amr/lidar_sensor.h"
#include "amr/backend_ws_client.h"

namespace amr {
namespace ros {

class SensorDataPublisher : public rclcpp::Node {
public:
    SensorDataPublisher(std::shared_ptr<amr::BackendWsClient> backend,
                        rclcpp::Node::SharedPtr parent);

private:
    void timerCallback();

    std::shared_ptr<amr::LidarSensor> lidar_;
    std::shared_ptr<amr::BackendWsClient> backend_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}
}
