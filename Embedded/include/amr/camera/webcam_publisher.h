#ifndef AMR_WEBCAM_PUBLISHER_H
#define AMR_WEBCAM_PUBLISHER_H

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

namespace amr {

class WebcamPublisher : public rclcpp::Node {
public:
  WebcamPublisher();

private:
  void timerCallback();

  cv::VideoCapture cap_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  

#endif
