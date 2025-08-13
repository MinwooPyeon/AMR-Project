#include "amr/ros/imu_data_publisher.h"
#include "amr/i2c/MyI2CImplementation.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

namespace amr {
namespace ros {

IMUDataPublisher::IMUDataPublisher(
    std::shared_ptr<BackendWsClient> backend,
    rclcpp::Node::SharedPtr parent_node)
    : Node("imu_data_publisher"), backend_(backend)
{
    // 파라미터 설정
    this->declare_parameter("frame_id", "imu_link");
    this->declare_parameter("publish_rate", 50.0);
    this->declare_parameter("enable_temperature", true);
    this->declare_parameter("imu_type", 4); // 0: MPU6050, 1: MPU9250, 2: BNO055, 3: LSM9DS1, 4: BNO08X
    this->declare_parameter("i2c_address", 0x4B);
    
    frame_id_ = this->get_parameter("frame_id").as_string();
    publish_rate_ = this->get_parameter("publish_rate").as_double();
    enable_temperature_ = this->get_parameter("enable_temperature").as_bool();
    int imu_type = this->get_parameter("imu_type").as_int();
    uint8_t i2c_address = static_cast<uint8_t>(this->get_parameter("i2c_address").as_int());
    
    // IMU 센서 초기화
    try {
        auto i2c = std::make_shared<MyI2CImplementation>("/dev/i2c-1");
        IMUType type = static_cast<IMUType>(imu_type);
        imu_ = std::make_shared<amr::IMUSensor>(i2c, type, i2c_address, "IMU_Sensor");
        
        if (!imu_->initialize()) {
            RCLCPP_ERROR(this->get_logger(), "IMU 센서 초기화 실패");
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "IMU 센서 초기화 완료");
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "IMU 센서 생성 실패: %s", e.what());
        return;
    }
    
    // 퍼블리셔 생성
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data", 10);
    if (enable_temperature_) {
        temp_pub_ = this->create_publisher<sensor_msgs::msg::Temperature>("/imu/temperature", 10);
    }
    
    // 타이머 설정
    auto timer_period = std::chrono::duration<double>(1.0 / publish_rate_);
    timer_ = this->create_wall_timer(
        timer_period,
        std::bind(&IMUDataPublisher::timerCallback, this)
    );
    
    RCLCPP_INFO(this->get_logger(), "IMU 데이터 퍼블리셔 초기화 완료 (주파수: %.1f Hz)", publish_rate_);
}

void IMUDataPublisher::timerCallback() {
    if (!imu_ || !imu_->isConnected()) {
        RCLCPP_WARN(this->get_logger(), "IMU 센서가 연결되지 않았습니다");
        return;
    }
    
    // IMU 데이터 읽기
    if (!imu_->readData()) {
        RCLCPP_WARN(this->get_logger(), "IMU 데이터 읽기 실패");
        return;
    }
    
    // 데이터 퍼블리시
    publishIMUData();
    if (enable_temperature_) {
        publishTemperatureData();
    }
}

void IMUDataPublisher::publishIMUData() {
    auto imu_data = imu_->getLatestData();
    
    auto msg = sensor_msgs::msg::Imu();
    msg.header.stamp = this->now();
    msg.header.frame_id = frame_id_;
    
    // 선형 가속도 (m/s²)
    msg.linear_acceleration.x = imu_data.accel_x;
    msg.linear_acceleration.y = imu_data.accel_y;
    msg.linear_acceleration.z = imu_data.accel_z;
    
    // 각속도 (rad/s)
    msg.angular_velocity.x = imu_data.gyro_x * M_PI / 180.0; // 도/초 -> rad/s
    msg.angular_velocity.y = imu_data.gyro_y * M_PI / 180.0;
    msg.angular_velocity.z = imu_data.gyro_z * M_PI / 180.0;
    
    // 자력계 데이터가 있는 경우
    if (imu_data.mag_x != 0.0f || imu_data.mag_y != 0.0f || imu_data.mag_z != 0.0f) {
        // 자력계 데이터를 orientation으로 변환 (간단한 구현)
        // 실제로는 Madgwick나 Mahony 필터를 사용해야 함
    }
    
    // 쿼터니언이 있는 경우
    if (imu_data.quaternion_w != 0.0f) {
        msg.orientation.w = imu_data.quaternion_w;
        msg.orientation.x = imu_data.quaternion_x;
        msg.orientation.y = imu_data.quaternion_y;
        msg.orientation.z = imu_data.quaternion_z;
    } else {
        // 기본값 (정면 방향)
        msg.orientation.w = 1.0;
        msg.orientation.x = 0.0;
        msg.orientation.y = 0.0;
        msg.orientation.z = 0.0;
    }
    
    // 공분산 행렬 (측정 불확실성)
    // 실제로는 센서 특성에 따라 조정 필요
    for (int i = 0; i < 9; i++) {
        msg.orientation_covariance[i] = 0.01;
        msg.angular_velocity_covariance[i] = 0.01;
        msg.linear_acceleration_covariance[i] = 0.01;
    }
    
    imu_pub_->publish(msg);
    
    // 백엔드로 로그 전송
    if (backend_) {
        std::string log_msg = "IMU Data - Accel: (" + 
                             std::to_string(imu_data.accel_x) + ", " +
                             std::to_string(imu_data.accel_y) + ", " +
                             std::to_string(imu_data.accel_z) + ") " +
                             "Gyro: (" + std::to_string(imu_data.gyro_x) + ", " +
                             std::to_string(imu_data.gyro_y) + ", " +
                             std::to_string(imu_data.gyro_z) + ")";
        backend_->sendLog(log_msg);
    }
    
    RCLCPP_DEBUG(this->get_logger(), "IMU 데이터 퍼블리시 완료");
}

void IMUDataPublisher::publishTemperatureData() {
    auto imu_data = imu_->getLatestData();
    
    auto msg = sensor_msgs::msg::Temperature();
    msg.header.stamp = this->now();
    msg.header.frame_id = frame_id_;
    msg.temperature = imu_data.temperature;
    msg.variance = 0.1; // 온도 측정 분산
    
    temp_pub_->publish(msg);
    
    RCLCPP_DEBUG(this->get_logger(), "온도 데이터 퍼블리시: %.2f°C", imu_data.temperature);
}

}
} 