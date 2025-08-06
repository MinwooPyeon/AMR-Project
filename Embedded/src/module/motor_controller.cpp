#include "amr/module/motor_controller.h"
#include <iostream>
#include <cmath>
#include <sstream>

namespace amr {

MotorController::MotorController(std::shared_ptr<MotorDriver> leftMotor,
                               std::shared_ptr<MotorDriver> rightMotor,
                               const std::string& name)
    : leftMotor_(std::move(leftMotor)), rightMotor_(std::move(rightMotor)), name_(name)
{
    if (!leftMotor_ || !rightMotor_) {
        status_ = RobotStatus::INITIALIZATION_ERROR;
        std::cerr << name_ << ": 모터 초기화 실패 - 모터 객체가 null입니다." << std::endl;
        return;
    }
    
    // 초기 상태 확인
    updateStatus();
    
    if (status_ == RobotStatus::OK) {
        stop(); // 안전을 위해 정지 상태로 초기화
        std::cout << name_ << ": 모터 컨트롤러 초기화 완료" << std::endl;
    } else {
        std::cerr << name_ << ": 모터 컨트롤러 초기화 실패 - " << getStatusString() << std::endl;
    }
}

MotorController::~MotorController() {
    emergencyStop();
}

bool MotorController::moveForward(int speed) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (status_ != RobotStatus::OK) {
        std::cerr << name_ << ": 전진 실패 - " << getStatusString() << std::endl;
        return false;
    }
    
    // Waveshare HAT은 하나의 드라이버로 양쪽 모터를 제어
    bool success = leftMotor_->setSpeed(speed, speed);
    
    if (success) {
        leftSpeed_ = speed;
        rightSpeed_ = speed;
        currentMovement_ = RobotMovement::FORWARD;
        logMovement(RobotMovement::FORWARD, speed);
        return true;
    } else {
        updateStatus();
        return false;
    }
}

bool MotorController::moveBackward(int speed) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (status_ != RobotStatus::OK) {
        std::cerr << name_ << ": 후진 실패 - " << getStatusString() << std::endl;
        return false;
    }
    
    // Waveshare HAT은 하나의 드라이버로 양쪽 모터를 제어
    bool success = leftMotor_->setSpeed(-speed, -speed);
    
    if (success) {
        leftSpeed_ = -speed;
        rightSpeed_ = -speed;
        currentMovement_ = RobotMovement::BACKWARD;
        logMovement(RobotMovement::BACKWARD, speed);
        return true;
    } else {
        updateStatus();
        return false;
    }
}

bool MotorController::turnLeft(int speed) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (status_ != RobotStatus::OK) {
        std::cerr << name_ << ": 좌회전 실패 - " << getStatusString() << std::endl;
        return false;
    }
    
    // 좌회전: 왼쪽 모터는 느리게, 오른쪽 모터는 빠르게
    int leftSpeed = speed * 0.7;  // 왼쪽 모터 70% 속도
    int rightSpeed = speed;       // 오른쪽 모터 100% 속도
    
    bool success = leftMotor_->setSpeed(leftSpeed, rightSpeed);
    
    if (success) {
        leftSpeed_ = leftSpeed;
        rightSpeed_ = rightSpeed;
        currentMovement_ = RobotMovement::TURN_LEFT;
        logMovement(RobotMovement::TURN_LEFT, speed);
        return true;
    } else {
        updateStatus();
        return false;
    }
}

bool MotorController::turnRight(int speed) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (status_ != RobotStatus::OK) {
        std::cerr << name_ << ": 우회전 실패 - " << getStatusString() << std::endl;
        return false;
    }
    
    // 우회전: 왼쪽 모터는 빠르게, 오른쪽 모터는 느리게
    int leftSpeed = speed;        // 왼쪽 모터 100% 속도
    int rightSpeed = speed * 0.7; // 오른쪽 모터 70% 속도
    
    bool success = leftMotor_->setSpeed(leftSpeed, rightSpeed);
    
    if (success) {
        leftSpeed_ = leftSpeed;
        rightSpeed_ = rightSpeed;
        currentMovement_ = RobotMovement::TURN_RIGHT;
        logMovement(RobotMovement::TURN_RIGHT, speed);
        return true;
    } else {
        updateStatus();
        return false;
    }
}

bool MotorController::rotateLeft(int speed) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (status_ != RobotStatus::OK) {
        std::cerr << name_ << ": 좌회전 실패 - " << getStatusString() << std::endl;
        return false;
    }
    
    // 좌회전: 왼쪽 모터는 후진, 오른쪽 모터는 전진
    bool success = leftMotor_->setSpeed(-speed, speed);
    
    if (success) {
        leftSpeed_ = -speed;
        rightSpeed_ = speed;
        currentMovement_ = RobotMovement::ROTATE_LEFT;
        logMovement(RobotMovement::ROTATE_LEFT, speed);
        return true;
    } else {
        updateStatus();
        return false;
    }
}

bool MotorController::rotateRight(int speed) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (status_ != RobotStatus::OK) {
        std::cerr << name_ << ": 우회전 실패 - " << getStatusString() << std::endl;
        return false;
    }
    
    // 우회전: 왼쪽 모터는 전진, 오른쪽 모터는 후진
    bool success = leftMotor_->setSpeed(speed, -speed);
    
    if (success) {
        leftSpeed_ = speed;
        rightSpeed_ = -speed;
        currentMovement_ = RobotMovement::ROTATE_RIGHT;
        logMovement(RobotMovement::ROTATE_RIGHT, speed);
        return true;
    } else {
        updateStatus();
        return false;
    }
}

bool MotorController::stop() {
    std::lock_guard<std::mutex> lock(mutex_);
    
    bool success = leftMotor_->stop();
    
    if (success) {
        leftSpeed_ = 0;
        rightSpeed_ = 0;
        currentMovement_ = RobotMovement::STOP;
        logMovement(RobotMovement::STOP, 0);
        return true;
    } else {
        updateStatus();
        return false;
    }
}

bool MotorController::emergencyStop() {
    std::lock_guard<std::mutex> lock(mutex_);
    
    bool success = leftMotor_->emergencyStop();
    
    leftSpeed_ = 0;
    rightSpeed_ = 0;
    currentMovement_ = RobotMovement::STOP;
    
    if (success) {
        std::cout << name_ << ": 비상 정지 완료" << std::endl;
        return true;
    } else {
        updateStatus();
        std::cerr << name_ << ": 비상 정지 중 오류 발생" << std::endl;
        return false;
    }
}

bool MotorController::moveWithDifferential(int leftSpeed, int rightSpeed) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (status_ != RobotStatus::OK) {
        std::cerr << name_ << ": 차동 이동 실패 - " << getStatusString() << std::endl;
        return false;
    }
    
    bool success = leftMotor_->setSpeed(leftSpeed, rightSpeed);
    
    if (success) {
        leftSpeed_ = leftSpeed;
        rightSpeed_ = rightSpeed;
        currentMovement_ = RobotMovement::STOP; // 차동 이동은 별도 카테고리
        return true;
    } else {
        updateStatus();
        return false;
    }
}

bool MotorController::moveWithCurvature(int speed, double curvature) {
    // 곡률 기반 이동: curvature가 양수면 우회전, 음수면 좌회전
    // curvature 범위: -1.0 ~ 1.0
    
    if (curvature < -1.0) curvature = -1.0;
    if (curvature > 1.0) curvature = 1.0;
    
    // 곡률에 따른 좌우 모터 속도 계산
    double leftSpeed = speed * (1.0 - curvature);
    double rightSpeed = speed * (1.0 + curvature);
    
    return moveWithDifferential(static_cast<int>(leftSpeed), static_cast<int>(rightSpeed));
}

RobotStatus MotorController::getStatus() const {
    return status_;
}

bool MotorController::isConnected() const {
    return status_ == RobotStatus::OK;
}

std::string MotorController::getStatusString() const {
    return statusToString(status_);
}

bool MotorController::setLeftMotorSpeed(int speed) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (status_ != RobotStatus::OK) {
        return false;
    }
    
    bool success = leftMotor_->setMotorASpeed(speed);
    if (success) {
        leftSpeed_ = speed;
    } else {
        updateStatus();
    }
    return success;
}

bool MotorController::setRightMotorSpeed(int speed) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (status_ != RobotStatus::OK) {
        return false;
    }
    
    bool success = leftMotor_->setMotorBSpeed(speed);
    if (success) {
        rightSpeed_ = speed;
    } else {
        updateStatus();
    }
    return success;
}

int MotorController::getLeftMotorSpeed() const {
    return leftSpeed_;
}

int MotorController::getRightMotorSpeed() const {
    return rightSpeed_;
}

bool MotorController::testMotors() {
    std::lock_guard<std::mutex> lock(mutex_);
    
    std::cout << name_ << ": 모터 테스트 시작..." << std::endl;
    
    // Waveshare HAT은 하나의 드라이버로 양쪽 모터를 제어
    bool test = leftMotor_->testConnection();
    
    if (test) {
        std::cout << name_ << ": 모터 테스트 성공" << std::endl;
        updateStatus();
        return true;
    } else {
        std::cerr << name_ << ": 모터 테스트 실패" << std::endl;
        updateStatus();
        return false;
    }
}

bool MotorController::resetMotors() {
    std::lock_guard<std::mutex> lock(mutex_);
    
    std::cout << name_ << ": 모터 리셋 시작..." << std::endl;
    
    bool reset = leftMotor_->reset();
    
    if (reset) {
        stop();
        updateStatus();
        std::cout << name_ << ": 모터 리셋 완료" << std::endl;
        return true;
    } else {
        std::cerr << name_ << ": 모터 리셋 실패" << std::endl;
        updateStatus();
        return false;
    }
}

bool MotorController::calibrateMotors() {
    std::lock_guard<std::mutex> lock(mutex_);
    
    std::cout << name_ << ": 모터 캘리브레이션 시작..." << std::endl;
    
    // 간단한 캘리브레이션: 양쪽 모터를 같은 속도로 테스트
    bool success = true;
    
    // 전진 테스트
    success &= moveForward(30);
    if (success) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        success &= stop();
    }
    
    // 후진 테스트
    if (success) {
        success &= moveBackward(30);
        if (success) {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            success &= stop();
        }
    }
    
    if (success) {
        std::cout << name_ << ": 모터 캘리브레이션 완료" << std::endl;
    } else {
        std::cerr << name_ << ": 모터 캘리브레이션 실패" << std::endl;
    }
    
    return success;
}

bool MotorController::setMotorAddresses(uint8_t leftAddr, uint8_t rightAddr) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    // Waveshare HAT은 하나의 주소만 사용
    bool success = leftMotor_->setI2CAddress(leftAddr);
    
    if (success) {
        std::cout << name_ << ": 모터 주소 변경 완료 - 0x" << std::hex << (int)leftAddr << std::dec << std::endl;
        return true;
    } else {
        std::cerr << name_ << ": 모터 주소 변경 실패" << std::endl;
        return false;
    }
}

std::pair<uint8_t, uint8_t> MotorController::getMotorAddresses() const {
    // Waveshare HAT은 하나의 주소만 사용
    uint8_t addr = leftMotor_->getI2CAddress();
    return std::make_pair(addr, addr);
}

void MotorController::updateStatus() {
    bool motorOK = (leftMotor_->getStatus() == MotorStatus::OK);
    
    if (motorOK) {
        status_ = RobotStatus::OK;
    } else {
        status_ = RobotStatus::BOTH_MOTORS_ERROR;
    }
}

bool MotorController::checkMotorConnection() {
    return leftMotor_->isConnected();
}

void MotorController::logMovement(RobotMovement movement, int speed) {
    std::cout << name_ << ": " << movementToString(movement) 
              << " (속도: " << speed << ")" << std::endl;
}

std::string MotorController::movementToString(RobotMovement movement) const {
    switch (movement) {
        case RobotMovement::STOP: return "정지";
        case RobotMovement::FORWARD: return "전진";
        case RobotMovement::BACKWARD: return "후진";
        case RobotMovement::TURN_LEFT: return "좌회전";
        case RobotMovement::TURN_RIGHT: return "우회전";
        case RobotMovement::ROTATE_LEFT: return "좌회전";
        case RobotMovement::ROTATE_RIGHT: return "우회전";
        default: return "알 수 없음";
    }
}

std::string MotorController::statusToString(RobotStatus status) const {
    switch (status) {
        case RobotStatus::OK: return "정상";
        case RobotStatus::LEFT_MOTOR_ERROR: return "왼쪽 모터 오류";
        case RobotStatus::RIGHT_MOTOR_ERROR: return "오른쪽 모터 오류";
        case RobotStatus::BOTH_MOTORS_ERROR: return "양쪽 모터 오류";
        case RobotStatus::INITIALIZATION_ERROR: return "초기화 오류";
        default: return "알 수 없는 오류";
    }
}

} // namespace amr 