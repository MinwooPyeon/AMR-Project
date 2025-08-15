#include "amr/angle_controller.h"
#include <iostream>
#include <cmath>
#include <algorithm>
#include <chrono>
#include <thread>

namespace amr {

AngleController::AngleController(std::shared_ptr<MotorController> motorController,
                               std::shared_ptr<IMUSensor> imuSensor,
                               const std::string& name)
    : motorController_(std::move(motorController))
    , imuSensor_(std::move(imuSensor))
    , name_(name)
    , lastPIDTime_(std::chrono::steady_clock::now())
{
    if (!motorController_ || !imuSensor_) {
        status_ = AngleControlStatus::ERROR;
        std::cerr << name_ << ": Initialization failed - Motor controller or IMU sensor is null." << std::endl;
        return;
    }
    
    std::cout << name_ << ": Rotation control controller creation completed" << std::endl;
}

AngleController::~AngleController() {
    stopControlLoop();
    stop();
}

bool AngleController::initialize() {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!motorController_ || !imuSensor_) {
        status_ = AngleControlStatus::ERROR;
        std::cerr << name_ << ": Initialization failed - Required components are missing." << std::endl;
        return false;
    }
    
    if (!imuSensor_->initialize()) {
        status_ = AngleControlStatus::ERROR;
        std::cerr << name_ << ": IMU sensor initialization failed" << std::endl;
        return false;
    }
    
    if (!motorController_->isConnected()) {
        status_ = AngleControlStatus::ERROR;
        std::cerr << name_ << ": Motor controller connection failed" << std::endl;
        return false;
    }
    
    IMUData imuData;
    if (imuSensor_->readData(imuData)) {
        currentAngle_ = imuData.yaw;
        targetAngle_ = currentAngle_;
        initialAngle_ = currentAngle_;
        smoothedAngle_ = currentAngle_;
        std::cout << name_ << ": Initial angle set - " << currentAngle_ << " degrees" << std::endl;
    }
    
    status_ = AngleControlStatus::IDLE;
    std::cout << name_ << ": Rotation control controller initialization completed" << std::endl;
    return true;
}

bool AngleController::calibrate() {
    std::lock_guard<std::mutex> lock(mutex_);
    
    status_ = AngleControlStatus::CALIBRATING;
    std::cout << name_ << ": controller calibration started" << std::endl;
    
    motorController_->stop();
    
    if (!imuSensor_->calibrate()) {
        status_ = AngleControlStatus::ERROR;
        std::cerr << name_ << ": IMU calibration failed" << std::endl;
        return false;
    }
    
    IMUData imuData;
    if (imuSensor_->readData(imuData)) {
        currentAngle_ = imuData.yaw;
        targetAngle_ = currentAngle_;
        initialAngle_ = currentAngle_;
        smoothedAngle_ = currentAngle_;
        std::cout << name_ << ": After calibration, initial angle - " << currentAngle_ << " degrees" << std::endl;
    }
    
    status_ = AngleControlStatus::IDLE;
    std::cout << name_ << ": Rotation control controller calibration completed" << std::endl;
    return true;
}

void AngleController::setConfig(const AngleControlConfig& config) {
    std::lock_guard<std::mutex> lock(mutex_);
    config_ = config;
    std::cout << name_ << ": Rotation control settings updated" << std::endl;
}

AngleControlConfig AngleController::getConfig() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return config_;
}

bool AngleController::turnLeft90() {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (status_ == AngleControlStatus::ERROR) {
        std::cerr << name_ << ": Left turn 90 degrees failed - Error state" << std::endl;
        return false;
    }
    
    IMUData imuData;
    if (imuSensor_->readData(imuData)) {
        initialAngle_ = imuData.yaw;
        currentAngle_ = initialAngle_;
    }
    
    targetAngle_ = normalizeAngle(initialAngle_ + 90.0);
    status_ = AngleControlStatus::TURNING_LEFT;
    
    logControl("Left turn 90 degrees started - initial: " + std::to_string(initialAngle_) + 
               " degrees, target: " + std::to_string(targetAngle_) + " degrees");
    return true;
}

bool AngleController::turnRight90() {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (status_ == AngleControlStatus::ERROR) {
        std::cerr << name_ << ": Right turn 90 degrees failed - Error state" << std::endl;
        return false;
    }
    
    IMUData imuData;
    if (imuSensor_->readData(imuData)) {
        initialAngle_ = imuData.yaw;
        currentAngle_ = initialAngle_;
    }
    
    targetAngle_ = normalizeAngle(initialAngle_ - 90.0);
    status_ = AngleControlStatus::TURNING_RIGHT;
    
    logControl("Right turn 90 degrees started - initial: " + std::to_string(initialAngle_) + 
               " degrees, target: " + std::to_string(targetAngle_) + " degrees");
    return true;
}

bool AngleController::turn180() {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (status_ == AngleControlStatus::ERROR) {
        std::cerr << name_ << ": 180 degree rotation failed - Error state" << std::endl;
        return false;
    }
    
    IMUData imuData;
    if (imuSensor_->readData(imuData)) {
        initialAngle_ = imuData.yaw;
        currentAngle_ = initialAngle_;
    }
    
    targetAngle_ = normalizeAngle(initialAngle_ + 180.0);
    status_ = AngleControlStatus::TURNING_180;
    
    logControl("Rotation 180 degrees started - initial: " + std::to_string(initialAngle_) + 
               " degrees, target: " + std::to_string(targetAngle_) + " degrees");
    return true;
}

bool AngleController::turnToAngle(double targetAngle) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (status_ == AngleControlStatus::ERROR) {
        std::cerr << name_ << ": Specific angle rotation failed - Error state" << std::endl;
        return false;
    }
    
    IMUData imuData;
    if (imuSensor_->readData(imuData)) {
        initialAngle_ = imuData.yaw;
        currentAngle_ = initialAngle_;
    }
    
    targetAngle_ = normalizeAngle(targetAngle);
    
    double angleDiff = calculateAngleError(currentAngle_, targetAngle_);
    if (angleDiff > 0) {
        status_ = AngleControlStatus::TURNING_LEFT;
        logControl("Left turn to specific angle - target: " + std::to_string(targetAngle_) + " degrees");
    } else {
        status_ = AngleControlStatus::TURNING_RIGHT;
        logControl("Right turn to specific angle - target: " + std::to_string(targetAngle_) + " degrees");
    }
    
    return true;
}

bool AngleController::stop() {
    std::lock_guard<std::mutex> lock(mutex_);
    
    motorController_->stop();
    status_ = AngleControlStatus::IDLE;
    
    logControl("stop");
    return true;
}

AngleControlStatus AngleController::getStatus() const {
    return status_;
}

bool AngleController::isActive() const {
    return status_ != AngleControlStatus::IDLE && status_ != AngleControlStatus::ERROR;
}

AngleControlResult AngleController::getCurrentResult() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return currentResult_;
}

double AngleController::getCurrentAngle() const {
    return currentAngle_;
}

double AngleController::getTargetAngle() const {
    return targetAngle_;
}

void AngleController::startControlLoop() {
    if (controlLoopRunning_) {
        std::cout << name_ << ": Control loop is already running" << std::endl;
        return;
    }
    
    shouldStopLoop_ = false;
    controlLoopThread_ = std::thread(&AngleController::controlLoop, this);
    controlLoopRunning_ = true;
    
    std::cout << name_ << ": Rotation control loop started" << std::endl;
}

void AngleController::stopControlLoop() {
    if (!controlLoopRunning_) {
        return;
    }
    
    shouldStopLoop_ = true;
    if (controlLoopThread_.joinable()) {
        controlLoopThread_.join();
    }
    controlLoopRunning_ = false;
    
    std::cout << name_ << ": Rotation control loop stopped" << std::endl;
}

bool AngleController::isControlLoopRunning() const {
    return controlLoopRunning_;
}

void AngleController::setPIDGains(double kp, double ki, double kd) {
    std::lock_guard<std::mutex> lock(mutex_);
    config_.pidKp = kp;
    config_.pidKi = ki;
    config_.pidKd = kd;
    std::cout << name_ << ": PID gains set - Kp:" << kp << " Ki:" << ki << " Kd:" << kd << std::endl;
}

void AngleController::enablePID(bool enable) {
    std::lock_guard<std::mutex> lock(mutex_);
    config_.enablePID = enable;
    std::cout << name_ << ": PID control " << (enable ? "enabled" : "disabled") << std::endl;
}

void AngleController::resetPID() {
    std::lock_guard<std::mutex> lock(mutex_);
    pidIntegral_ = 0.0;
    pidPreviousError_ = 0.0;
    lastPIDTime_ = std::chrono::steady_clock::now();
    std::cout << name_ << ": PID controller reset" << std::endl;
}

void AngleController::enableSmoothing(bool enable) {
    std::lock_guard<std::mutex> lock(mutex_);
    config_.enableSmoothing = enable;
    std::cout << name_ << ": Angle smoothing " << (enable ? "enabled" : "disabled") << std::endl;
}

void AngleController::setSmoothingFactor(double factor) {
    std::lock_guard<std::mutex> lock(mutex_);
    config_.smoothingFactor = std::clamp(factor, 0.0, 1.0);
    std::cout << name_ << ": Smoothing factor set - " << config_.smoothingFactor << std::endl;
}

bool AngleController::testRotationControl() {
    std::cout << name_ << ": Rotation control test started..." << std::endl;
    
    IMUData imuData;
    if (!imuSensor_->readData(imuData)) {
        std::cerr << name_ << ": IMU data read failed" << std::endl;
        return false;
    }
    
    std::cout << name_ << ": Current angle - " << imuData.yaw << " degrees" << std::endl;
    
    bool success = true;
    
    success &= turnLeft90();
    if (success) {
        std::this_thread::sleep_for(std::chrono::seconds(5));
        success &= stop();
    }
    
    if (success) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        success &= turnRight90();
        if (success) {
            std::this_thread::sleep_for(std::chrono::seconds(5));
            success &= stop();
        }
    }
    
    if (success) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        success &= turn180();
        if (success) {
            std::this_thread::sleep_for(std::chrono::seconds(8));
            success &= stop();
        }
    }
    
    if (success) {
        std::cout << name_ << ": Rotation control test successful" << std::endl;
    } else {
        std::cerr << name_ << ": Rotation control test failed" << std::endl;
    }
    
    return success;
}

void AngleController::printStatus() const {
    std::cout << "=== " << name_ << " Status ===" << std::endl;
    std::cout << "Status: " << static_cast<int>(status_) << std::endl;
    std::cout << "Current angle: " << currentAngle_ << " degrees" << std::endl;
    std::cout << "Target angle: " << targetAngle_ << " degrees" << std::endl;
    std::cout << "Initial angle: " << initialAngle_ << " degrees" << std::endl;
    std::cout << "Angle error: " << angleError_ << " degrees" << std::endl;
    std::cout << "Control loop running: " << (controlLoopRunning_ ? "Yes" : "No") << std::endl;
    std::cout << "==================" << std::endl;
}

void AngleController::controlLoop() {
    const auto controlPeriod = std::chrono::microseconds(
        static_cast<long>(1000000.0 / config_.controlFrequency)
    );
    
    while (!shouldStopLoop_) {
        auto startTime = std::chrono::steady_clock::now();
        
        IMUData imuData;
        if (imuSensor_->readData(imuData)) {
            double newAngle = config_.enableSmoothing ? 
                smoothAngle(imuData.yaw) : imuData.yaw;
            
            currentAngle_ = newAngle;
            angleError_ = calculateAngleError(currentAngle_, targetAngle_);
            
            if (!checkSafetyConditions()) {
                emergencyStop();
                continue;
            }
            
            if (status_ == AngleControlStatus::TURNING_LEFT || 
                status_ == AngleControlStatus::TURNING_RIGHT ||
                status_ == AngleControlStatus::TURNING_180) {
                
                if (std::abs(angleError_) > config_.angleTolerance) {
                    double controlOutput = config_.enablePID ? 
                        calculatePIDOutput(angleError_, 1.0 / config_.controlFrequency) :
                        angleError_ * config_.pidKp;
                    
                    if (status_ == AngleControlStatus::TURNING_LEFT) {
                        motorController_->rotateLeftSingleWheel(static_cast<int>(config_.turnSpeed));
                    } else {
                        motorController_->rotateRightSingleWheel(static_cast<int>(config_.turnSpeed));
                    }
                } else {
                    motorController_->stop();
                    status_ = AngleControlStatus::IDLE;
                    logControl("Target angle reached - " + std::to_string(targetAngle_) + " degrees");
                }
            }
            
            updateCurrentResult();
        }
        
        auto elapsed = std::chrono::steady_clock::now() - startTime;
        if (elapsed < controlPeriod) {
            std::this_thread::sleep_for(controlPeriod - elapsed);
        }
    }
}

double AngleController::calculatePIDOutput(double error, double dt) {
    if (!config_.enablePID) {
        return error * config_.pidKp;
    }
    
    pidIntegral_ += error * dt;
    
    double derivative = (error - pidPreviousError_) / dt;
    
    double output = config_.pidKp * error + 
                   config_.pidKi * pidIntegral_ + 
                   config_.pidKd * derivative;
    
    pidIntegral_ = std::clamp(pidIntegral_, -100.0, 100.0);
    
    output = std::clamp(output, -100.0, 100.0);
    
    pidPreviousError_ = error;
    return output;
}

double AngleController::smoothAngle(double newAngle) {
    if (firstSmoothing_) {
        smoothedAngle_ = newAngle;
        firstSmoothing_ = false;
        return newAngle;
    }
    
    smoothedAngle_ = config_.smoothingFactor * smoothedAngle_ + 
                    (1.0 - config_.smoothingFactor) * newAngle;
    return smoothedAngle_;
}

double AngleController::normalizeAngle(double angle) {
    while (angle > 180.0) angle -= 360.0;
    while (angle < -180.0) angle += 360.0;
    return angle;
}

double AngleController::calculateAngleError(double current, double target) {
    double error = target - current;
    
    while (error > 180.0) error -= 360.0;
    while (error < -180.0) error += 360.0;
    
    return error;
}

void AngleController::updateCurrentResult() {
    currentResult_.success = (status_ != AngleControlStatus::ERROR);
    currentResult_.status = status_;
    currentResult_.currentAngle = currentAngle_;
    currentResult_.targetAngle = targetAngle_;
    currentResult_.angleError = angleError_;
    currentResult_.turnProgress = calculateTurnProgress();
    currentResult_.timestamp = std::chrono::steady_clock::now();
}

double AngleController::calculateTurnProgress() {
    if (status_ == AngleControlStatus::TURNING_LEFT || 
        status_ == AngleControlStatus::TURNING_RIGHT ||
        status_ == AngleControlStatus::TURNING_180) {
        
        double totalRotation = 0.0;
        if (status_ == AngleControlStatus::TURNING_180) {
            totalRotation = 180.0;
        } else {
            totalRotation = 90.0;
        }
        
        double completedRotation = std::abs(currentAngle_ - initialAngle_);
        return std::clamp(completedRotation / totalRotation, 0.0, 1.0);
    }
    return 0.0;
}

void AngleController::logControl(const std::string& message) {
    std::cout << name_ << ": " << message << std::endl;
}

void AngleController::emergencyStop() {
    motorController_->emergencyStop();
    status_ = AngleControlStatus::ERROR;
    std::cerr << name_ << ": Emergency stop executed" << std::endl;
}

bool AngleController::checkSafetyConditions() {
    if (std::abs(angleError_) > 180.0) {
        std::cerr << name_ << ": Angle error is too large - " << angleError_ << " degrees" << std::endl;
        return false;
    }
    
    if (!motorController_->isConnected()) {
        std::cerr << name_ << ": Motor controller connection lost" << std::endl;
        return false;
    }
    
    return true;
}

}