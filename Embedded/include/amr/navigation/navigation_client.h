#ifndef AMR_NAVIGATION_CLIENT_H
#define AMR_NAVIGATION_CLIENT_H

#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace amr {

class NavigationClient : public rclcpp::Node {
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  NavigationClient();

  void sendGoal(float x, float y, float yaw);

private:
  rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
};

} 

#endif
