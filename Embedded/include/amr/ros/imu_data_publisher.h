#ifndef AMR_IMU_DATA_PUBLISHER_H
#define AMR_IMU_DATA_PUBLISHER_H

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "amr/imu_sensor.h"
#include "amr/backend_ws_client.h"

namespace amr {
namespace ros {

class IMUDataPublisher : public rclcpp::Node {
public:
    IMUDataPublisher(std::shared_ptr<amr::BackendWsClient> backend,
                     rclcpp::Node::SharedPtr parent);

private:
    void timerCallback();
    void publishIMUData();
    void publishTemperatureData();

    std::shared_ptr<amr::IMUSensor> imu_;
    std::shared_ptr<amr::BackendWsClient> backend_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temp_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    std::string frame_id_;
    double publish_rate_;
    bool enable_temperature_;
};

}
}

#endif 