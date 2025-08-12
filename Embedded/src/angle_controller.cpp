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
        std::cerr << name_ << ": 초기화 실패 - 모터 컨트롤러 또는 IMU 센서가 null입니다." << std::endl;
        return;
    }
    
    std::cout << name_ << ": 회전 제어 컨트롤러 생성 완료" << std::endl;
}

AngleController::~AngleController() {
    stopControlLoop();
    stop();
}

bool AngleController::initialize() {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!motorController_ || !imuSensor_) {
        status_ = AngleControlStatus::ERROR;
        std::cerr << name_ << ": 초기화 실패 - 필수 컴포넌트가 없습니다." << std::endl;
        return false;
    }
    
    // IMU 센서 초기화
    if (!imuSensor_->initialize()) {
        status_ = AngleControlStatus::ERROR;
        std::cerr << name_ << ": IMU 센서 초기화 실패" << std::endl;
        return false;
    }
    
    // 모터 컨트롤러 상태 확인
    if (!motorController_->isConnected()) {
        status_ = AngleControlStatus::ERROR;
        std::cerr << name_ << ": 모터 컨트롤러 연결 실패" << std::endl;
        return false;
    }
    
    // 초기 각도 읽기
    IMUData imuData;
    if (imuSensor_->readData(imuData)) {
        currentAngle_ = imuData.yaw;
        targetAngle_ = currentAngle_;
        initialAngle_ = currentAngle_;
        smoothedAngle_ = currentAngle_;
        std::cout << name_ << ": 초기 각도 설정 - " << currentAngle_ << "도" << std::endl;
    }
    
    status_ = AngleControlStatus::IDLE;
    std::cout << name_ << ": 회전 제어 컨트롤러 초기화 완료" << std::endl;
    return true;
}

bool AngleController::calibrate() {
    std::lock_guard<std::mutex> lock(mutex_);
    
    status_ = AngleControlStatus::CALIBRATING;
    std::cout << name_ << ": 회전 제어 컨트롤러 캘리브레이션 시작..." << std::endl;
    
    // 모터 정지
    motorController_->stop();
    
    // IMU 캘리브레이션
    if (!imuSensor_->calibrate()) {
        status_ = AngleControlStatus::ERROR;
        std::cerr << name_ << ": IMU 캘리브레이션 실패" << std::endl;
        return false;
    }
    
    // 초기 각도 재설정
    IMUData imuData;
    if (imuSensor_->readData(imuData)) {
        currentAngle_ = imuData.yaw;
        targetAngle_ = currentAngle_;
        initialAngle_ = currentAngle_;
        smoothedAngle_ = currentAngle_;
        std::cout << name_ << ": 캘리브레이션 후 초기 각도 - " << currentAngle_ << "도" << std::endl;
    }
    
    status_ = AngleControlStatus::IDLE;
    std::cout << name_ << ": 회전 제어 컨트롤러 캘리브레이션 완료" << std::endl;
    return true;
}

void AngleController::setConfig(const AngleControlConfig& config) {
    std::lock_guard<std::mutex> lock(mutex_);
    config_ = config;
    std::cout << name_ << ": 회전 제어 설정 업데이트" << std::endl;
}

AngleControlConfig AngleController::getConfig() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return config_;
}

bool AngleController::turnLeft90() {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (status_ == AngleControlStatus::ERROR) {
        std::cerr << name_ << ": 좌회전 90도 실패 - 오류 상태" << std::endl;
        return false;
    }
    
    // 현재 각도를 초기 각도로 설정
    IMUData imuData;
    if (imuSensor_->readData(imuData)) {
        initialAngle_ = imuData.yaw;
        currentAngle_ = initialAngle_;
    }
    
    // 목표 각도 계산 (좌회전 90도)
    targetAngle_ = normalizeAngle(initialAngle_ + 90.0);
    status_ = AngleControlStatus::TURNING_LEFT;
    
    logControl("좌회전 90도 시작 - 초기: " + std::to_string(initialAngle_) + 
               "도, 목표: " + std::to_string(targetAngle_) + "도");
    return true;
}

bool AngleController::turnRight90() {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (status_ == AngleControlStatus::ERROR) {
        std::cerr << name_ << ": 우회전 90도 실패 - 오류 상태" << std::endl;
        return false;
    }
    
    // 현재 각도를 초기 각도로 설정
    IMUData imuData;
    if (imuSensor_->readData(imuData)) {
        initialAngle_ = imuData.yaw;
        currentAngle_ = initialAngle_;
    }
    
    // 목표 각도 계산 (우회전 90도)
    targetAngle_ = normalizeAngle(initialAngle_ - 90.0);
    status_ = AngleControlStatus::TURNING_RIGHT;
    
    logControl("우회전 90도 시작 - 초기: " + std::to_string(initialAngle_) + 
               "도, 목표: " + std::to_string(targetAngle_) + "도");
    return true;
}

bool AngleController::turn180() {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (status_ == AngleControlStatus::ERROR) {
        std::cerr << name_ << ": 회전 180도 실패 - 오류 상태" << std::endl;
        return false;
    }
    
    // 현재 각도를 초기 각도로 설정
    IMUData imuData;
    if (imuSensor_->readData(imuData)) {
        initialAngle_ = imuData.yaw;
        currentAngle_ = initialAngle_;
    }
    
    // 목표 각도 계산 (회전 180도)
    targetAngle_ = normalizeAngle(initialAngle_ + 180.0);
    status_ = AngleControlStatus::TURNING_180;
    
    logControl("회전 180도 시작 - 초기: " + std::to_string(initialAngle_) + 
               "도, 목표: " + std::to_string(targetAngle_) + "도");
    return true;
}

bool AngleController::turnToAngle(double targetAngle) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (status_ == AngleControlStatus::ERROR) {
        std::cerr << name_ << ": 특정 각도 회전 실패 - 오류 상태" << std::endl;
        return false;
    }
    
    // 현재 각도를 초기 각도로 설정
    IMUData imuData;
    if (imuSensor_->readData(imuData)) {
        initialAngle_ = imuData.yaw;
        currentAngle_ = initialAngle_;
    }
    
    targetAngle_ = normalizeAngle(targetAngle);
    
    // 회전 방향 결정
    double angleDiff = calculateAngleError(currentAngle_, targetAngle_);
    if (angleDiff > 0) {
        status_ = AngleControlStatus::TURNING_LEFT;
        logControl("좌회전으로 특정 각도 이동 - 목표: " + std::to_string(targetAngle_) + "도");
    } else {
        status_ = AngleControlStatus::TURNING_RIGHT;
        logControl("우회전으로 특정 각도 이동 - 목표: " + std::to_string(targetAngle_) + "도");
    }
    
    return true;
}

bool AngleController::stop() {
    std::lock_guard<std::mutex> lock(mutex_);
    
    motorController_->stop();
    status_ = AngleControlStatus::IDLE;
    
    logControl("회전 제어 정지");
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
        std::cout << name_ << ": 제어 루프가 이미 실행 중입니다." << std::endl;
        return;
    }
    
    shouldStopLoop_ = false;
    controlLoopThread_ = std::thread(&AngleController::controlLoop, this);
    controlLoopRunning_ = true;
    
    std::cout << name_ << ": 회전 제어 루프 시작" << std::endl;
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
    
    std::cout << name_ << ": 회전 제어 루프 정지" << std::endl;
}

bool AngleController::isControlLoopRunning() const {
    return controlLoopRunning_;
}

void AngleController::setPIDGains(double kp, double ki, double kd) {
    std::lock_guard<std::mutex> lock(mutex_);
    config_.pidKp = kp;
    config_.pidKi = ki;
    config_.pidKd = kd;
    std::cout << name_ << ": PID 게인 설정 - Kp:" << kp << " Ki:" << ki << " Kd:" << kd << std::endl;
}

void AngleController::enablePID(bool enable) {
    std::lock_guard<std::mutex> lock(mutex_);
    config_.enablePID = enable;
    std::cout << name_ << ": PID 제어 " << (enable ? "활성화" : "비활성화") << std::endl;
}

void AngleController::resetPID() {
    std::lock_guard<std::mutex> lock(mutex_);
    pidIntegral_ = 0.0;
    pidPreviousError_ = 0.0;
    lastPIDTime_ = std::chrono::steady_clock::now();
    std::cout << name_ << ": PID 제어기 리셋" << std::endl;
}

void AngleController::enableSmoothing(bool enable) {
    std::lock_guard<std::mutex> lock(mutex_);
    config_.enableSmoothing = enable;
    std::cout << name_ << ": 각도 스무딩 " << (enable ? "활성화" : "비활성화") << std::endl;
}

void AngleController::setSmoothingFactor(double factor) {
    std::lock_guard<std::mutex> lock(mutex_);
    config_.smoothingFactor = std::clamp(factor, 0.0, 1.0);
    std::cout << name_ << ": 스무딩 팩터 설정 - " << config_.smoothingFactor << std::endl;
}

bool AngleController::testRotationControl() {
    std::cout << name_ << ": 회전 제어 테스트 시작..." << std::endl;
    
    // 현재 각도 읽기
    IMUData imuData;
    if (!imuSensor_->readData(imuData)) {
        std::cerr << name_ << ": IMU 데이터 읽기 실패" << std::endl;
        return false;
    }
    
    std::cout << name_ << ": 현재 각도 - " << imuData.yaw << "도" << std::endl;
    
    // 회전 테스트
    bool success = true;
    
    // 좌회전 90도 테스트
    success &= turnLeft90();
    if (success) {
        std::this_thread::sleep_for(std::chrono::seconds(5));
        success &= stop();
    }
    
    // 우회전 90도 테스트
    if (success) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        success &= turnRight90();
        if (success) {
            std::this_thread::sleep_for(std::chrono::seconds(5));
            success &= stop();
        }
    }
    
    // 180도 회전 테스트
    if (success) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        success &= turn180();
        if (success) {
            std::this_thread::sleep_for(std::chrono::seconds(8));
            success &= stop();
        }
    }
    
    if (success) {
        std::cout << name_ << ": 회전 제어 테스트 성공" << std::endl;
    } else {
        std::cerr << name_ << ": 회전 제어 테스트 실패" << std::endl;
    }
    
    return success;
}

void AngleController::printStatus() const {
    std::cout << "=== " << name_ << " 상태 ===" << std::endl;
    std::cout << "상태: " << static_cast<int>(status_) << std::endl;
    std::cout << "현재 각도: " << currentAngle_ << "도" << std::endl;
    std::cout << "목표 각도: " << targetAngle_ << "도" << std::endl;
    std::cout << "초기 각도: " << initialAngle_ << "도" << std::endl;
    std::cout << "각도 오차: " << angleError_ << "도" << std::endl;
    std::cout << "제어 루프 실행: " << (controlLoopRunning_ ? "예" : "아니오") << std::endl;
    std::cout << "==================" << std::endl;
}

void AngleController::controlLoop() {
    const auto controlPeriod = std::chrono::microseconds(
        static_cast<long>(1000000.0 / config_.controlFrequency)
    );
    
    while (!shouldStopLoop_) {
        auto startTime = std::chrono::steady_clock::now();
        
        // IMU 데이터 읽기
        IMUData imuData;
        if (imuSensor_->readData(imuData)) {
            // 각도 스무딩
            double newAngle = config_.enableSmoothing ? 
                smoothAngle(imuData.yaw) : imuData.yaw;
            
            currentAngle_ = newAngle;
            angleError_ = calculateAngleError(currentAngle_, targetAngle_);
            
            // 안전 조건 확인
            if (!checkSafetyConditions()) {
                emergencyStop();
                continue;
            }
            
            // 회전 제어 로직
            if (status_ == AngleControlStatus::TURNING_LEFT || 
                status_ == AngleControlStatus::TURNING_RIGHT ||
                status_ == AngleControlStatus::TURNING_180) {
                
                if (std::abs(angleError_) > config_.angleTolerance) {
                    double controlOutput = config_.enablePID ? 
                        calculatePIDOutput(angleError_, 1.0 / config_.controlFrequency) :
                        angleError_ * config_.pidKp;
                    
                    // 회전 방향에 따른 모터 제어 (휠 1개만 사용)
                    if (status_ == AngleControlStatus::TURNING_LEFT) {
                        // 좌회전: 오른쪽 휠만 전진
                        motorController_->rotateLeftSingleWheel(static_cast<int>(config_.turnSpeed));
                    } else {
                        // 우회전: 왼쪽 휠만 전진
                        motorController_->rotateRightSingleWheel(static_cast<int>(config_.turnSpeed));
                    }
                } else {
                    // 목표 각도 도달
                    motorController_->stop();
                    status_ = AngleControlStatus::IDLE;
                    logControl("목표 각도 도달 - " + std::to_string(targetAngle_) + "도");
                }
            }
            
            // 결과 업데이트
            updateCurrentResult();
        }
        
        // 제어 주파수 유지
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
    
    // 적분 항
    pidIntegral_ += error * dt;
    
    // 미분 항
    double derivative = (error - pidPreviousError_) / dt;
    
    // PID 출력 계산
    double output = config_.pidKp * error + 
                   config_.pidKi * pidIntegral_ + 
                   config_.pidKd * derivative;
    
    // 적분 와인드업 방지
    pidIntegral_ = std::clamp(pidIntegral_, -100.0, 100.0);
    
    // 출력 제한
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
    
    // 각도 차이를 -180 ~ 180 범위로 정규화
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
    std::cerr << name_ << ": 비상 정지 실행" << std::endl;
}

bool AngleController::checkSafetyConditions() {
    // 각도 오차가 너무 클 때
    if (std::abs(angleError_) > 180.0) {
        std::cerr << name_ << ": 각도 오차가 너무 큽니다 - " << angleError_ << "도" << std::endl;
        return false;
    }
    
    // 모터 컨트롤러 상태 확인
    if (!motorController_->isConnected()) {
        std::cerr << name_ << ": 모터 컨트롤러 연결 끊김" << std::endl;
        return false;
    }
    
    return true;
}

} // namespace amr 