#pragma once
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include "amr/angle_controller.h"
#include "amr/module/motor_controller.h"
#include "amr/imu_sensor.h"

namespace amr {

class AngleControlNode : public rclcpp::Node {
public:
    AngleControlNode();
    ~AngleControlNode();

private:
    // ROS2 컴포넌트
    std::shared_ptr<MotorController> motorController_;
    std::shared_ptr<IMUSensor> imuSensor_;
    std::unique_ptr<AngleController> angleController_;
    
    // ROS2 퍼블리셔
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imuPublisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr anglePublisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr targetAnglePublisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr angleErrorPublisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr statusPublisher_;
    
    // ROS2 서브스크라이버
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdVelSubscription_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr targetAngleSubscription_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enableSubscription_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr commandSubscription_;
    
    // ROS2 타이머
    rclcpp::TimerBase::SharedPtr controlTimer_;
    rclcpp::TimerBase::SharedPtr statusTimer_;
    
    // ROS2 서비스
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enableService_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr calibrateService_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr resetService_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr turnLeft90Service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr turnRight90Service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr turn180Service_;
    
    // 상태 변수
    std::atomic<bool> angleControlEnabled_{false};
    std::atomic<bool> isInitialized_{false};
    std::atomic<double> currentAngle_{0.0};
    std::atomic<double> targetAngle_{0.0};
    std::atomic<double> angleError_{0.0};
    
    // 콜백 함수
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void targetAngleCallback(const std_msgs::msg::Float64::SharedPtr msg);
    void enableCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void commandCallback(const std_msgs::msg::String::SharedPtr msg);
    
    void controlTimerCallback();
    void statusTimerCallback();
    
    bool enableServiceCallback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                             std::shared_ptr<std_srvs::srv::SetBool::Response> response);
    bool calibrateServiceCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    bool resetServiceCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                            std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    bool turnLeft90ServiceCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                 std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    bool turnRight90ServiceCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                  std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    bool turn180ServiceCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                              std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    
    // 초기화 및 설정
    bool initializeComponents();
    bool loadParameters();
    void setupPublishers();
    void setupSubscribers();
    void setupServices();
    void setupTimers();
    
    // 회전 제어 명령 처리
    void processRotationCommand(const std::string& command);
    void processTurnCommand(const std::string& direction);
    
    // 안전 및 상태 관리
    void emergencyStop();
    void publishStatus();
    void publishIMUData();
    void publishAngleData();
    
    // 유틸리티 함수
    std::string statusToString(AngleControlStatus status);
    void logInfo(const std::string& message);
    void logError(const std::string& message);
    void logWarn(const std::string& message);
};

} // namespace amr 