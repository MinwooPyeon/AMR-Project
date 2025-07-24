#include "amr/navigation/navigation_client.h"

namespace amr {

NavigationClient::NavigationClient()
: Node("navigation_client")
{
  client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "navigate_to_pose");
}

void NavigationClient::sendGoal(float x, float y, float yaw) {
  if (!client_->wait_for_action_server(std::chrono::seconds(5))) {
    RCLCPP_ERROR(this->get_logger(), "Failed to connect to Nav2 action server");
    return;
  }

  auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
  goal_msg.pose.pose.position.x = x;
  goal_msg.pose.pose.position.y = y;
  goal_msg.pose.pose.orientation.z = sin(yaw / 2);
  goal_msg.pose.pose.orientation.w = cos(yaw / 2);

  auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
  send_goal_options.result_callback = [this](auto result) {
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_INFO(this->get_logger(), "Navigation succeeded");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Navigation failed");
    }
  };

  client_->async_send_goal(goal_msg, send_goal_options);
}

}  
