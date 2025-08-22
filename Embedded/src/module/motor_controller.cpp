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
        std::cerr << name_ << ": Motor initialization failed - Motor object is null." << std::endl;
        return;
    }
    

    updateStatus();
    
    if (status_ == RobotStatus::OK) {
        stop();
        std::cout << name_ << ": Motor controller initialization completed" << std::endl;
    } else {
        std::cerr << name_ << ": Motor controller initialization failed - " << getStatusString() << std::endl;
    }
}

MotorController::~MotorController() {
    emergencyStop();
}

bool MotorController::moveForward(int speed) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (status_ != RobotStatus::OK) {
        std::cerr << name_ << ": Forward movement failed - " << getStatusString() << std::endl;
        return false;
    }
    

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
        std::cerr << name_ << ": Backward movement failed - " << getStatusString() << std::endl;
        return false;
    }
    

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
        std::cerr << name_ << ": Left turn failed - " << getStatusString() << std::endl;
        return false;
    }
    
    int leftSpeed = speed * 0.7;
    int rightSpeed = speed;
    
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
        std::cerr << name_ << ": Right turn failed - " << getStatusString() << std::endl;
        return false;
    }
    
    int leftSpeed = speed;
    int rightSpeed = speed * 0.7;
    
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
        std::cerr << name_ << ": Left rotation failed - " << getStatusString() << std::endl;
        return false;
    }
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
        std::cerr << name_ << ": Right rotation failed - " << getStatusString() << std::endl;
        return false;
    }
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

bool MotorController::rotateLeftSingleWheel(int speed) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (status_ != RobotStatus::OK) {
        std::cerr << name_ << ": Single wheel left rotation failed - " << getStatusString() << std::endl;
        return false;
    }
    bool success = leftMotor_->setSpeed(0, speed);
    
    if (success) {
        leftSpeed_ = 0;
        rightSpeed_ = speed;
        currentMovement_ = RobotMovement::SINGLE_WHEEL_ROTATE_LEFT;
        logMovement(RobotMovement::SINGLE_WHEEL_ROTATE_LEFT, speed);
        return true;
    } else {
        updateStatus();
        return false;
    }
}

bool MotorController::rotateRightSingleWheel(int speed) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (status_ != RobotStatus::OK) {
        std::cerr << name_ << ": Single wheel right rotation failed - " << getStatusString() << std::endl;
        return false;
    }
    bool success = leftMotor_->setSpeed(speed, 0);
    
    if (success) {
        leftSpeed_ = speed;
        rightSpeed_ = 0;
        currentMovement_ = RobotMovement::SINGLE_WHEEL_ROTATE_RIGHT;
        logMovement(RobotMovement::SINGLE_WHEEL_ROTATE_RIGHT, speed);
        return true;
    } else {
        updateStatus();
        return false;
    }
}

bool MotorController::rotateLeftSingleWheelLeft(int speed) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (status_ != RobotStatus::OK) {
        std::cerr << name_ << ": Left wheel left rotation failed - " << getStatusString() << std::endl;
        return false;
    }
    bool success = leftMotor_->setSpeed(-speed, 0);
    
    if (success) {
        leftSpeed_ = -speed;
        rightSpeed_ = 0;
        currentMovement_ = RobotMovement::SINGLE_WHEEL_ROTATE_LEFT;
        logMovement(RobotMovement::SINGLE_WHEEL_ROTATE_LEFT, speed);
        return true;
    } else {
        updateStatus();
        return false;
    }
}

bool MotorController::rotateRightSingleWheelRight(int speed) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (status_ != RobotStatus::OK) {
        std::cerr << name_ << ": Right wheel right rotation failed - " << getStatusString() << std::endl;
        return false;
    }
    bool success = leftMotor_->setSpeed(0, -speed);
    
    if (success) {
        leftSpeed_ = 0;
        rightSpeed_ = -speed;
        currentMovement_ = RobotMovement::SINGLE_WHEEL_ROTATE_RIGHT;
        logMovement(RobotMovement::SINGLE_WHEEL_ROTATE_RIGHT, speed);
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
        std::cout << name_ << ": Emergency stop completed" << std::endl;
        return true;
    } else {
        updateStatus();
        std::cerr << name_ << ": Error occurred during emergency stop" << std::endl;
        return false;
    }
}

bool MotorController::moveWithDifferential(int leftSpeed, int rightSpeed) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (status_ != RobotStatus::OK) {
        std::cerr << name_ << ": Differential movement failed - " << getStatusString() << std::endl;
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

    // curvature 범위: -1.0 ~ 1.0
    
    if (curvature < -1.0) curvature = -1.0;
    if (curvature > 1.0) curvature = 1.0;
    

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
    
    std::cout << name_ << ": Motor test started..." << std::endl;
    

    bool test = leftMotor_->testConnection();
    
    if (test) {
        std::cout << name_ << ": Motor test successful" << std::endl;
        updateStatus();
        return true;
    } else {
        std::cerr << name_ << ": Motor test failed" << std::endl;
        updateStatus();
        return false;
    }
}

bool MotorController::resetMotors() {
    std::lock_guard<std::mutex> lock(mutex_);
    
    std::cout << name_ << ": Motor reset started..." << std::endl;
    
    bool reset = leftMotor_->reset();
    
    if (reset) {
        stop();
        updateStatus();
        std::cout << name_ << ": Motor reset completed" << std::endl;
        return true;
    } else {
        std::cerr << name_ << ": Motor reset failed" << std::endl;
        updateStatus();
        return false;
    }
}

bool MotorController::calibrateMotors() {
    std::lock_guard<std::mutex> lock(mutex_);
    
    std::cout << name_ << ": Motor calibration started..." << std::endl;
    

    bool success = true;
    

    success &= moveForward(30);
    if (success) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        success &= stop();
    }
    

    if (success) {
        success &= moveBackward(30);
        if (success) {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            success &= stop();
        }
    }
    
    if (success) {
        std::cout << name_ << ": Motor calibration completed" << std::endl;
    } else {
        std::cerr << name_ << ": Motor calibration failed" << std::endl;
    }
    
    return success;
}

bool MotorController::setMotorAddresses(uint8_t leftAddr, uint8_t rightAddr) {
    std::lock_guard<std::mutex> lock(mutex_);
    

    bool success = leftMotor_->setI2CAddress(leftAddr);
    
    if (success) {
        std::cout << name_ << ": Motor address change completed - 0x" << std::hex << (int)leftAddr << std::dec << std::endl;
        return true;
    } else {
        std::cerr << name_ << ": Motor address change failed" << std::endl;
        return false;
    }
}

std::pair<uint8_t, uint8_t> MotorController::getMotorAddresses() const {

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
              << " (speed: " << speed << ")" << std::endl;
}

std::string MotorController::movementToString(RobotMovement movement) const {
    switch (movement) {
        case RobotMovement::STOP: return "Stop";
        case RobotMovement::FORWARD: return "Forward";
        case RobotMovement::BACKWARD: return "Backward";
        case RobotMovement::TURN_LEFT: return "Turn Left";
        case RobotMovement::TURN_RIGHT: return "Turn Right";
        case RobotMovement::ROTATE_LEFT: return "Rotate Left (Both Wheels)";
        case RobotMovement::ROTATE_RIGHT: return "Rotate Right (Both Wheels)";
        case RobotMovement::SINGLE_WHEEL_ROTATE_LEFT: return "Rotate Left (Single Wheel)";
        case RobotMovement::SINGLE_WHEEL_ROTATE_RIGHT: return "Rotate Right (Single Wheel)";
        default: return "Unknown";
    }
}

std::string MotorController::statusToString(RobotStatus status) const {
    switch (status) {
        case RobotStatus::OK: return "Normal";
        case RobotStatus::LEFT_MOTOR_ERROR: return "Left Motor Error";
        case RobotStatus::RIGHT_MOTOR_ERROR: return "Right Motor Error";
        case RobotStatus::BOTH_MOTORS_ERROR: return "Both Motors Error";
        case RobotStatus::INITIALIZATION_ERROR: return "Initialization Error";
        default: return "Unknown Error";
    }
}

} // namespace amr 