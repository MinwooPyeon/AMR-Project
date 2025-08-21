#include <rclcpp/rclcpp.hpp>

#include "amr/backend_ws_client.h"
#include "amr/mqtt/robot_data_publisher.h"
#include <sensor_msgs/msg/battery_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <vector>
#include <string>
#include <atomic>
#include <mutex>
#include <thread>

using std::placeholders::_1;

std::atomic<float> g_battery_pct{0.0f};
std::string g_motor_state = "STOP";
std::mutex g_motor_mtx;

std::vector<float> g_lidar_ranges;
std::mutex g_lidar_mtx;



void battery_cb(const sensor_msgs::msg::BatteryState::SharedPtr msg) {
    g_battery_pct = msg->percentage;
}

void motor_cb(const std_msgs::msg::String::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(g_motor_mtx);
    g_motor_state = msg->data;
}

void lidar_cb(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(g_lidar_mtx);
    g_lidar_ranges = msg->ranges;
}



int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("amr_main");
    auto logger = rclcpp::get_logger("amr_main");

    auto ws_client = std::make_shared<amr::BackendWsClient>("ws://192.168.0.1:8080/ws");

    auto robot_data_publisher = std::make_shared<RobotDataPublisher>(
        "amr_robot_publisher", 
        "192.168.100.141", 
        "robot_data", 
        1883, 
        60
    );

    ws_client->setRecvHandler([&](const std::string& payload){
        Json::Value root;
        Json::Reader reader;
        if (reader.parse(payload, root) && root.isMember("type") && root["type"].asString() == "cmd") {
            std::string cmd = root["cmd"].asString();
            RCLCPP_INFO(logger, "[AI CMD] Received: %s", cmd.c_str());
        } else {
            RCLCPP_WARN(logger, "Unknown message received: %s", payload.c_str());
        }
    });

    auto battery_sub = node->create_subscription<sensor_msgs::msg::BatteryState>(
        "/battery_state", 10, battery_cb);
    auto motor_sub = node->create_subscription<std_msgs::msg::String>(
        "/motor_state", 10, motor_cb);
    auto lidar_sub = node->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, lidar_cb);


    std::atomic<bool> running{true};
    std::thread sender_thread([&]() {
        while (running) {
            float battery = g_battery_pct.load();
            std::string motor;
            {
                std::lock_guard<std::mutex> lock(g_motor_mtx);
                motor = g_motor_state;
            }
            std::vector<float> lidar;
            {
                std::lock_guard<std::mutex> lock(g_lidar_mtx);
                lidar = g_lidar_ranges;
            }
            ws_client->sendState(battery, motor);
            ws_client->sendLidar(lidar);
            ws_client->sendLog("AMR status transmission completed");
            
            robot_data_publisher->publishRobotData(
                "AMR001",
                motor,
                10.5,
                20.3,
                1.5,
                45.0
            );
            
            std::this_thread::sleep_for(std::chrono::seconds(2));
        }
    });

    rclcpp::spin(node);

    running = false;
    sender_thread.join();
    rclcpp::shutdown();
    return 0;
}
