#ifndef AMR_LIDAR_SENSOR_H
#define AMR_LIDAR_SENSOR_H

#include <string>
#include <mutex>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace amr {

class LidarSensor {
public:
  LidarSensor(rclcpp::Node::SharedPtr node, const std::string& topic = "/scan");

  void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

  float getDistance();  
  void update();    

private:
  std::string topic_;
  float distance_;
  std::mutex mutex_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
};

}  

#endif
