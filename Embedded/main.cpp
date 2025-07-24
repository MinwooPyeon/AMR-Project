#include "rclcpp/rclcpp.hpp"
#include "amr/backend_ws_client.h"
#include "amr/ros/sensor_data_publisher.h"
#include "amr/ros/motor_cmd_subscriber.h"
#include "amr/battery/battery_publisher.h"
#include "amr/camera/webcam_publisher.h"
#include "amr/slam/visual_lidar_slam_node.h"
#include "amr/navigation/navigation_client.h"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto main_node = rclcpp::Node::make_shared("amr_main");

    auto backend_ws = std::make_shared<amr::BackendWsClient>("ws://192.168.0.1:8080/ws");

    backend_ws->sendLog("🤖 Robot system has started.");

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(std::make_shared<amr::ros::SensorDataPublisher>(backend_ws, main_node));
    executor.add_node(std::make_shared<amr::ros::MotorCmdSubscriber>(main_node));
    executor.add_node(std::make_shared<amr::BatteryPublisher>());
    executor.add_node(std::make_shared<amr::WebcamPublisher>());
    executor.add_node(std::make_shared<amr::VisualLidarSlamNode>());
    executor.add_node(std::make_shared<amr::NavigationClient>());
    executor.add_node(main_node); 

    executor.spin();
    rclcpp::shutdown();
    return 0;
}
