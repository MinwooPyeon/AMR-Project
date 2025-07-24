#include "amr/camera/webcam_publisher.h"

namespace amr {

WebcamPublisher::WebcamPublisher()
: Node("webcam_publisher")
{
  cap_.open(0);
  if (!cap_.isOpened()) {
    RCLCPP_ERROR(this->get_logger(), "Cannot open webcam");
    throw std::runtime_error("Webcam open error");
  }

  image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("camera/image_raw", 10);
  timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&WebcamPublisher::timerCallback, this));
}

void WebcamPublisher::timerCallback() {
  cv::Mat frame;
  cap_ >> frame;
  if (frame.empty()) {
    RCLCPP_WARN(this->get_logger(), "Empty frame captured");
    return;
  }
  auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
  image_pub_->publish(*msg);
}

}  
