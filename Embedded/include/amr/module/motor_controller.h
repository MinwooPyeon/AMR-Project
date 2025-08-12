#pragma once
#include <memory>
#include <string>
#include <atomic>
#include <mutex>
#include "motor_driver.h"

namespace amr {

enum class RobotMovement {
    STOP = 0,
    FORWARD = 1,
    BACKWARD = 2,
    TURN_LEFT = 3,
    TURN_RIGHT = 4,
    ROTATE_LEFT = 5,
    ROTATE_RIGHT = 6,
    SINGLE_WHEEL_ROTATE_LEFT = 7,   // 휠 1개만 사용한 좌회전
    SINGLE_WHEEL_ROTATE_RIGHT = 8   // 휠 1개만 사용한 우회전
};

enum class RobotStatus {
    OK = 0,
    LEFT_MOTOR_ERROR = 1,
    RIGHT_MOTOR_ERROR = 2,
    BOTH_MOTORS_ERROR = 3,
    INITIALIZATION_ERROR = 4
};

class MotorController {
public:
    MotorController(std::shared_ptr<MotorDriver> leftMotor,
                   std::shared_ptr<MotorDriver> rightMotor,
                   const std::string& name = "MotorController");
    ~MotorController();

    // 기본 이동 명령
    bool moveForward(int speed);
    bool moveBackward(int speed);
    bool turnLeft(int speed);
    bool turnRight(int speed);
    bool rotateLeft(int speed);
    bool rotateRight(int speed);
    bool stop();
    bool emergencyStop();

    // 휠 1개만 사용한 회전 명령
    bool rotateLeftSingleWheel(int speed);    // 왼쪽 휠만 사용하여 좌회전
    bool rotateRightSingleWheel(int speed);   // 오른쪽 휠만 사용하여 우회전
    bool rotateLeftSingleWheelLeft(int speed);   // 왼쪽 휠만 사용하여 좌회전
    bool rotateRightSingleWheelRight(int speed); // 오른쪽 휠만 사용하여 우회전

    // 고급 이동 명령
    bool moveWithDifferential(int leftSpeed, int rightSpeed);
    bool moveWithCurvature(int speed, double curvature); // 곡률 기반 이동
    
    // 상태 확인
    RobotStatus getStatus() const;
    bool isConnected() const;
    std::string getStatusString() const;
    
    // 모터 개별 제어
    bool setLeftMotorSpeed(int speed);
    bool setRightMotorSpeed(int speed);
    int getLeftMotorSpeed() const;
    int getRightMotorSpeed() const;
    
    // 설정 및 진단
    bool testMotors();
    bool resetMotors();
    bool calibrateMotors();
    
    // I2C 주소 관리
    bool setMotorAddresses(uint8_t leftAddr, uint8_t rightAddr);
    std::pair<uint8_t, uint8_t> getMotorAddresses() const;

private:
    std::shared_ptr<MotorDriver> leftMotor_;
    std::shared_ptr<MotorDriver> rightMotor_;
    std::string name_;
    
    mutable std::mutex mutex_;
    std::atomic<RobotStatus> status_{RobotStatus::OK};
    std::atomic<int> leftSpeed_{0};
    std::atomic<int> rightSpeed_{0};
    std::atomic<RobotMovement> currentMovement_{RobotMovement::STOP};
    
    // 내부 헬퍼 함수
    void updateStatus();
    bool checkMotorConnection();
    void logMovement(RobotMovement movement, int speed);
    std::string movementToString(RobotMovement movement) const;
    std::string statusToString(RobotStatus status) const;
};

} // namespace amr 