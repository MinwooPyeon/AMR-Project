#include "amr/ros/angle_control_node.h"
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <iostream>
#include <sstream>

namespace amr {

AngleControlNode::AngleControlNode()
    : Node("angle_control_node")
{
    logInfo("회전 제어 노드 시작");
    
    // 파라미터 로드
    if (!loadParameters()) {
        logError("파라미터 로드 실패");
        return;
    }
    
    // 컴포넌트 초기화
    if (!initializeComponents()) {
        logError("컴포넌트 초기화 실패");
        return;
    }
    
    // ROS2 인터페이스 설정
    setupPublishers();
    setupSubscribers();
    setupServices();
    setupTimers();
    
    isInitialized_ = true;
    logInfo("회전 제어 노드 초기화 완료");
}

AngleControlNode::~AngleControlNode() {
    if (angleController_) {
        angleController_->stopControlLoop();
        angleController_->stop();
    }
    logInfo("회전 제어 노드 종료");
}

bool AngleControlNode::loadParameters() {
    // IMU 센서 파라미터
    this->declare_parameter("imu_type", "BNO08X");
    this->declare_parameter("imu_i2c_address", 0x4B);
    this->declare_parameter("imu_sample_rate", 100);
    
    // 모터 컨트롤러 파라미터
    this->declare_parameter("motor_i2c_address", 0x40);
    this->declare_parameter("motor_pwm_frequency", 50);
    
    // 회전 제어 파라미터
    this->declare_parameter("angle_tolerance", 2.0);
    this->declare_parameter("turn_speed", 50.0);
    this->declare_parameter("control_frequency", 50.0);
    this->declare_parameter("pid_kp", 2.0);
    this->declare_parameter("pid_ki", 0.1);
    this->declare_parameter("pid_kd", 0.5);
    this->declare_parameter("enable_smoothing", true);
    this->declare_parameter("smoothing_factor", 0.8);
    
    // ROS2 토픽 파라미터
    this->declare_parameter("cmd_vel_topic", "cmd_vel");
    this->declare_parameter("target_angle_topic", "target_angle");
    this->declare_parameter("enable_topic", "angle_control_enable");
    this->declare_parameter("command_topic", "angle_control_command");
    this->declare_parameter("imu_topic", "imu/data");
    this->declare_parameter("angle_topic", "current_angle");
    this->declare_parameter("target_angle_pub_topic", "target_angle_pub");
    this->declare_parameter("angle_error_topic", "angle_error");
    this->declare_parameter("status_topic", "angle_control_status");
    
    return true;
}

bool AngleControlNode::initializeComponents() {
    try {
        // IMU 센서 초기화
        int imuAddress = this->get_parameter("imu_i2c_address").as_int();
        std::string imuType = this->get_parameter("imu_type").as_string();
        
        imuSensor_ = std::make_shared<IMUSensor>(imuAddress, imuType);
        if (!imuSensor_->initialize()) {
            logError("IMU 센서 초기화 실패");
            return false;
        }
        
        // 모터 드라이버 초기화
        int motorAddress = this->get_parameter("motor_i2c_address").as_int();
        auto leftMotor = std::make_shared<MotorDriver>(motorAddress, "LeftMotor");
        auto rightMotor = std::make_shared<MotorDriver>(motorAddress, "RightMotor");
        
        if (!leftMotor->initialize() || !rightMotor->initialize()) {
            logError("모터 드라이버 초기화 실패");
            return false;
        }
        
        // 모터 컨트롤러 초기화
        motorController_ = std::make_shared<MotorController>(leftMotor, rightMotor, "AngleControlMotor");
        if (!motorController_->isConnected()) {
            logError("모터 컨트롤러 초기화 실패");
            return false;
        }
        
        // 각도 컨트롤러 초기화
        angleController_ = std::make_unique<AngleController>(motorController_, imuSensor_, "AngleController");
        
        // 회전 제어 설정
        AngleControlConfig config;
        config.angleTolerance = this->get_parameter("angle_tolerance").as_double();
        config.turnSpeed = this->get_parameter("turn_speed").as_double();
        config.controlFrequency = this->get_parameter("control_frequency").as_double();
        config.pidKp = this->get_parameter("pid_kp").as_double();
        config.pidKi = this->get_parameter("pid_ki").as_double();
        config.pidKd = this->get_parameter("pid_kd").as_double();
        config.enableSmoothing = this->get_parameter("enable_smoothing").as_bool();
        config.smoothingFactor = this->get_parameter("smoothing_factor").as_double();
        
        angleController_->setConfig(config);
        
        if (!angleController_->initialize()) {
            logError("각도 컨트롤러 초기화 실패");
            return false;
        }
        
        // 캘리브레이션 수행
        if (!angleController_->calibrate()) {
            logWarn("각도 컨트롤러 캘리브레이션 실패");
        }
        
        logInfo("모든 컴포넌트 초기화 완료");
        return true;
        
    } catch (const std::exception& e) {
        logError("컴포넌트 초기화 중 예외 발생: " + std::string(e.what()));
        return false;
    }
}

void AngleControlNode::setupPublishers() {
    std::string imuTopic = this->get_parameter("imu_topic").as_string();
    std::string angleTopic = this->get_parameter("angle_topic").as_string();
    std::string targetAnglePubTopic = this->get_parameter("target_angle_pub_topic").as_string();
    std::string angleErrorTopic = this->get_parameter("angle_error_topic").as_string();
    std::string statusTopic = this->get_parameter("status_topic").as_string();
    
    imuPublisher_ = this->create_publisher<sensor_msgs::msg::Imu>(imuTopic, 10);
    anglePublisher_ = this->create_publisher<std_msgs::msg::Float64>(angleTopic, 10);
    targetAnglePublisher_ = this->create_publisher<std_msgs::msg::Float64>(targetAnglePubTopic, 10);
    angleErrorPublisher_ = this->create_publisher<std_msgs::msg::Float64>(angleErrorTopic, 10);
    statusPublisher_ = this->create_publisher<std_msgs::msg::String>(statusTopic, 10);
    
    logInfo("퍼블리셔 설정 완료");
}

void AngleControlNode::setupSubscribers() {
    std::string cmdVelTopic = this->get_parameter("cmd_vel_topic").as_string();
    std::string targetAngleTopic = this->get_parameter("target_angle_topic").as_string();
    std::string enableTopic = this->get_parameter("enable_topic").as_string();
    std::string commandTopic = this->get_parameter("command_topic").as_string();
    
    cmdVelSubscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
        cmdVelTopic, 10, std::bind(&AngleControlNode::cmdVelCallback, this, std::placeholders::_1));
    
    targetAngleSubscription_ = this->create_subscription<std_msgs::msg::Float64>(
        targetAngleTopic, 10, std::bind(&AngleControlNode::targetAngleCallback, this, std::placeholders::_1));
    
    enableSubscription_ = this->create_subscription<std_msgs::msg::Bool>(
        enableTopic, 10, std::bind(&AngleControlNode::enableCallback, this, std::placeholders::_1));
    
    commandSubscription_ = this->create_subscription<std_msgs::msg::String>(
        commandTopic, 10, std::bind(&AngleControlNode::commandCallback, this, std::placeholders::_1));
    
    logInfo("서브스크라이버 설정 완료");
}

void AngleControlNode::setupServices() {
    enableService_ = this->create_service<std_srvs::srv::SetBool>(
        "angle_control_enable", 
        std::bind(&AngleControlNode::enableServiceCallback, this, std::placeholders::_1, std::placeholders::_2));
    
    calibrateService_ = this->create_service<std_srvs::srv::Trigger>(
        "angle_control_calibrate",
        std::bind(&AngleControlNode::calibrateServiceCallback, this, std::placeholders::_1, std::placeholders::_2));
    
    resetService_ = this->create_service<std_srvs::srv::Trigger>(
        "angle_control_reset",
        std::bind(&AngleControlNode::resetServiceCallback, this, std::placeholders::_1, std::placeholders::_2));
    
    turnLeft90Service_ = this->create_service<std_srvs::srv::Trigger>(
        "turn_left_90",
        std::bind(&AngleControlNode::turnLeft90ServiceCallback, this, std::placeholders::_1, std::placeholders::_2));
    
    turnRight90Service_ = this->create_service<std_srvs::srv::Trigger>(
        "turn_right_90",
        std::bind(&AngleControlNode::turnRight90ServiceCallback, this, std::placeholders::_1, std::placeholders::_2));
    
    turn180Service_ = this->create_service<std_srvs::srv::Trigger>(
        "turn_180",
        std::bind(&AngleControlNode::turn180ServiceCallback, this, std::placeholders::_1, std::placeholders::_2));
    
    logInfo("서비스 설정 완료");
}

void AngleControlNode::setupTimers() {
    double controlFreq = this->get_parameter("control_frequency").as_double();
    auto controlPeriod = std::chrono::milliseconds(static_cast<int>(1000.0 / controlFreq));
    
    controlTimer_ = this->create_wall_timer(controlPeriod, 
        std::bind(&AngleControlNode::controlTimerCallback, this));
    
    statusTimer_ = this->create_wall_timer(std::chrono::milliseconds(100), 
        std::bind(&AngleControlNode::statusTimerCallback, this));
    
    logInfo("타이머 설정 완료");
}

void AngleControlNode::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    if (!angleControlEnabled_ || !isInitialized_) {
        return;
    }
    
    // cmd_vel의 angular.z를 회전 명령으로 해석
    if (std::abs(msg->angular.z) > 0.1) {
        if (msg->angular.z > 0) {
            // 좌회전
            angleController_->turnLeft90();
        } else {
            // 우회전
            angleController_->turnRight90();
        }
    } else if (std::abs(msg->linear.x) > 0.1) {
        // 직진 명령 - 회전 제어에서는 무시
        logInfo("직진 명령 무시 (회전 제어 모드)");
    } else {
        // 정지 명령
        angleController_->stop();
    }
}

void AngleControlNode::targetAngleCallback(const std_msgs::msg::Float64::SharedPtr msg) {
    if (!angleControlEnabled_ || !isInitialized_) {
        return;
    }
    
    targetAngle_ = msg->data;
    angleController_->turnToAngle(targetAngle_);
    logInfo("목표 각도 설정: " + std::to_string(targetAngle_) + "도");
}

void AngleControlNode::enableCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    angleControlEnabled_ = msg->data;
    
    if (angleControlEnabled_) {
        if (isInitialized_) {
            angleController_->startControlLoop();
            logInfo("회전 제어 활성화");
        } else {
            logError("회전 제어 활성화 실패 - 초기화되지 않음");
        }
    } else {
        angleController_->stopControlLoop();
        angleController_->stop();
        logInfo("회전 제어 비활성화");
    }
}

void AngleControlNode::commandCallback(const std_msgs::msg::String::SharedPtr msg) {
    if (!angleControlEnabled_ || !isInitialized_) {
        logWarn("회전 제어가 비활성화되어 명령을 무시합니다");
        return;
    }
    
    processRotationCommand(msg->data);
}

void AngleControlNode::controlTimerCallback() {
    if (!isInitialized_ || !angleControlEnabled_) {
        return;
    }
    
    // IMU 데이터 읽기 및 발행
    IMUData imuData;
    if (imuSensor_->readData(imuData)) {
        currentAngle_ = imuData.yaw;
        publishIMUData();
        publishAngleData();
    }
}

void AngleControlNode::statusTimerCallback() {
    if (!isInitialized_) {
        return;
    }
    
    publishStatus();
}

bool AngleControlNode::enableServiceCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
    
    angleControlEnabled_ = request->data;
    
    if (angleControlEnabled_) {
        if (isInitialized_) {
            angleController_->startControlLoop();
            response->message = "회전 제어 활성화됨";
            logInfo("서비스를 통해 회전 제어 활성화");
        } else {
            response->message = "회전 제어 활성화 실패 - 초기화되지 않음";
            logError("서비스를 통해 회전 제어 활성화 실패");
        }
    } else {
        angleController_->stopControlLoop();
        angleController_->stop();
        response->message = "회전 제어 비활성화됨";
        logInfo("서비스를 통해 회전 제어 비활성화");
    }
    
    response->success = isInitialized_;
    return true;
}

bool AngleControlNode::calibrateServiceCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    
    if (!isInitialized_) {
        response->success = false;
        response->message = "초기화되지 않음";
        return true;
    }
    
    bool success = angleController_->calibrate();
    response->success = success;
    response->message = success ? "캘리브레이션 완료" : "캘리브레이션 실패";
    
    logInfo("캘리브레이션 서비스 호출: " + response->message);
    return true;
}

bool AngleControlNode::resetServiceCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    
    if (!isInitialized_) {
        response->success = false;
        response->message = "초기화되지 않음";
        return true;
    }
    
    angleController_->stop();
    angleController_->resetPID();
    targetAngle_ = currentAngle_;
    
    response->success = true;
    response->message = "리셋 완료";
    
    logInfo("리셋 서비스 호출: " + response->message);
    return true;
}

bool AngleControlNode::turnLeft90ServiceCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    
    if (!isInitialized_) {
        response->success = false;
        response->message = "초기화되지 않음";
        return true;
    }
    
    bool success = angleController_->turnLeft90();
    response->success = success;
    response->message = success ? "좌회전 90도 명령 실행" : "좌회전 90도 명령 실패";
    
    logInfo("좌회전 90도 서비스 호출: " + response->message);
    return true;
}

bool AngleControlNode::turnRight90ServiceCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    
    if (!isInitialized_) {
        response->success = false;
        response->message = "초기화되지 않음";
        return true;
    }
    
    bool success = angleController_->turnRight90();
    response->success = success;
    response->message = success ? "우회전 90도 명령 실행" : "우회전 90도 명령 실패";
    
    logInfo("우회전 90도 서비스 호출: " + response->message);
    return true;
}

bool AngleControlNode::turn180ServiceCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    
    if (!isInitialized_) {
        response->success = false;
        response->message = "초기화되지 않음";
        return true;
    }
    
    bool success = angleController_->turn180();
    response->success = success;
    response->message = success ? "회전 180도 명령 실행" : "회전 180도 명령 실패";
    
    logInfo("회전 180도 서비스 호출: " + response->message);
    return true;
}

void AngleControlNode::processRotationCommand(const std::string& command) {
    std::istringstream iss(command);
    std::string cmd;
    iss >> cmd;
    
    if (cmd == "turn_left_90" || cmd == "left_90") {
        angleController_->turnLeft90();
        logInfo("좌회전 90도 명령");
    } else if (cmd == "turn_right_90" || cmd == "right_90") {
        angleController_->turnRight90();
        logInfo("우회전 90도 명령");
    } else if (cmd == "turn_180" || cmd == "180") {
        angleController_->turn180();
        logInfo("회전 180도 명령");
    } else if (cmd == "stop") {
        angleController_->stop();
        logInfo("회전 제어 정지 명령");
    } else if (cmd == "calibrate") {
        angleController_->calibrate();
        logInfo("캘리브레이션 명령");
    } else {
        logWarn("알 수 없는 명령: " + command);
    }
}

void AngleControlNode::processTurnCommand(const std::string& direction) {
    if (direction == "left") {
        angleController_->turnLeft90();
        logInfo("좌회전 90도 명령");
    } else if (direction == "right") {
        angleController_->turnRight90();
        logInfo("우회전 90도 명령");
    } else if (direction == "180") {
        angleController_->turn180();
        logInfo("회전 180도 명령");
    }
}

void AngleControlNode::emergencyStop() {
    if (angleController_) {
        angleController_->stop();
    }
    if (motorController_) {
        motorController_->emergencyStop();
    }
    logError("비상 정지 실행");
}

void AngleControlNode::publishStatus() {
    if (!angleController_) {
        return;
    }
    
    auto statusMsg = std_msgs::msg::String();
    statusMsg.data = statusToString(angleController_->getStatus());
    statusPublisher_->publish(statusMsg);
}

void AngleControlNode::publishIMUData() {
    if (!imuSensor_) {
        return;
    }
    
    IMUData imuData;
    if (imuSensor_->readData(imuData)) {
        auto imuMsg = sensor_msgs::msg::Imu();
        imuMsg.header.stamp = this->now();
        imuMsg.header.frame_id = "imu_link";
        
        // 가속도 데이터
        imuMsg.linear_acceleration.x = imuData.accelX;
        imuMsg.linear_acceleration.y = imuData.accelY;
        imuMsg.linear_acceleration.z = imuData.accelZ;
        
        // 각속도 데이터
        imuMsg.angular_velocity.x = imuData.gyroX;
        imuMsg.angular_velocity.y = imuData.gyroY;
        imuMsg.angular_velocity.z = imuData.gyroZ;
        
        imuPublisher_->publish(imuMsg);
    }
}

void AngleControlNode::publishAngleData() {
    // 현재 각도 발행
    auto angleMsg = std_msgs::msg::Float64();
    angleMsg.data = currentAngle_;
    anglePublisher_->publish(angleMsg);
    
    // 목표 각도 발행
    auto targetMsg = std_msgs::msg::Float64();
    targetMsg.data = targetAngle_;
    targetAnglePublisher_->publish(targetMsg);
    
    // 각도 오차 발행
    auto errorMsg = std_msgs::msg::Float64();
    errorMsg.data = angleError_;
    angleErrorPublisher_->publish(errorMsg);
}

std::string AngleControlNode::statusToString(AngleControlStatus status) {
    switch (status) {
        case AngleControlStatus::IDLE: return "IDLE";
        case AngleControlStatus::TURNING_LEFT: return "TURNING_LEFT";
        case AngleControlStatus::TURNING_RIGHT: return "TURNING_RIGHT";
        case AngleControlStatus::TURNING_180: return "TURNING_180";
        case AngleControlStatus::CALIBRATING: return "CALIBRATING";
        case AngleControlStatus::ERROR: return "ERROR";
        default: return "UNKNOWN";
    }
}

void AngleControlNode::logInfo(const std::string& message) {
    RCLCPP_INFO(this->get_logger(), "[AngleControlNode] %s", message.c_str());
}

void AngleControlNode::logError(const std::string& message) {
    RCLCPP_ERROR(this->get_logger(), "[AngleControlNode] %s", message.c_str());
}

void AngleControlNode::logWarn(const std::string& message) {
    RCLCPP_WARN(this->get_logger(), "[AngleControlNode] %s", message.c_str());
}

} // namespace amr 