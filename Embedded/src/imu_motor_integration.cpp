#include "amr/module/motor_controller.h"
#include "amr/imu_sensor.h"
#include "amr/i2c/MyI2CImplementation.h"
#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>
#include <memory>
#include <atomic>
#include <mutex>

namespace amr {

class IMUMotorIntegration {
public:
    IMUMotorIntegration(const std::string& i2cDevice = "/dev/i2c-1")
        : i2cDevice_(i2cDevice)
        , isRunning_(false)
        , targetAngle_(0.0)
        , currentAngle_(0.0)
        , isTurning_(false)
        , turnDirection_(0) // 0: 정지, 1: 좌회전, -1: 우회전
    {
        std::cout << "IMU-모터 통합 시스템 초기화 시작" << std::endl;
    }

    ~IMUMotorIntegration() {
        stop();
    }

    bool initialize() {
        try {
            // I2C 인터페이스 초기화
            i2cInterface_ = std::make_shared<MyI2CImplementation>(i2cDevice_);
            
            // GY-BN008x IMU 초기화 (MPU6050 기반)
            imuSensor_ = std::make_shared<IMUSensor>(i2cInterface_, IMUType::MPU6050, 0x68, "GY-BN008x");
            
            if (!imuSensor_->initialize()) {
                std::cerr << "IMU 센서 초기화 실패" << std::endl;
                return false;
            }
            
            // IMU 설정
            imuSensor_->setSampleRate(100);  // 100Hz 샘플링
            imuSensor_->setGyroRange(250);   // ±250°/s
            imuSensor_->setAccelRange(2);    // ±2g
            imuSensor_->enableLowPassFilter(true);
            imuSensor_->setFilterCutoff(5.0f);
            
            // 모터 컨트롤러 초기화
            motorDriver_ = std::make_shared<MotorDriver>(i2cInterface_, 0x60, "MotorDriver");
            if (!motorDriver_->initialize()) {
                std::cerr << "모터 드라이버 초기화 실패" << std::endl;
                return false;
            }
            
            motorController_ = std::make_shared<MotorController>(motorDriver_, motorDriver_, "IMUMotorController");
            
            // 초기 각도 캘리브레이션
            if (!calibrateInitialAngle()) {
                std::cerr << "초기 각도 캘리브레이션 실패" << std::endl;
                return false;
            }
            
            std::cout << "IMU-모터 통합 시스템 초기화 완료" << std::endl;
            return true;
            
        } catch (const std::exception& e) {
            std::cerr << "초기화 중 오류 발생: " << e.what() << std::endl;
            return false;
        }
    }

    bool calibrateInitialAngle() {
        std::cout << "초기 각도 캘리브레이션 시작..." << std::endl;
        
        // 3초간 데이터 수집하여 평균 각도 계산
        double sumAngle = 0.0;
        int sampleCount = 0;
        const int targetSamples = 300; // 3초 * 100Hz
        
        auto startTime = std::chrono::steady_clock::now();
        
        while (sampleCount < targetSamples) {
            if (imuSensor_->readData()) {
                auto data = imuSensor_->getLatestData();
                sumAngle += data.yaw;
                sampleCount++;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        
        if (sampleCount > 0) {
            currentAngle_ = sumAngle / sampleCount;
            targetAngle_ = currentAngle_;
            std::cout << "초기 각도 캘리브레이션 완료: " << currentAngle_ << "도" << std::endl;
            return true;
        }
        
        return false;
    }

    void start() {
        if (isRunning_) {
            std::cout << "이미 실행 중입니다." << std::endl;
            return;
        }
        
        isRunning_ = true;
        controlThread_ = std::thread(&IMUMotorIntegration::controlLoop, this);
        std::cout << "IMU-모터 통합 제어 루프 시작" << std::endl;
    }

    void stop() {
        if (!isRunning_) return;
        
        isRunning_ = false;
        isTurning_ = false;
        turnDirection_ = 0;
        
        if (controlThread_.joinable()) {
            controlThread_.join();
        }
        
        if (motorController_) {
            motorController_->stop();
        }
        
        std::cout << "IMU-모터 통합 제어 루프 정지" << std::endl;
    }

    // 좌회전 90도
    bool turnLeft90() {
        if (!isRunning_) {
            std::cerr << "제어 루프가 실행되지 않았습니다." << std::endl;
            return false;
        }
        
        std::lock_guard<std::mutex> lock(controlMutex_);
        targetAngle_ = normalizeAngle(currentAngle_ + 90.0);
        isTurning_ = true;
        turnDirection_ = 1; // 좌회전
        
        std::cout << "좌회전 90도 시작: " << currentAngle_ << "도 → " << targetAngle_ << "도" << std::endl;
        return true;
    }

    // 우회전 90도
    bool turnRight90() {
        if (!isRunning_) {
            std::cerr << "제어 루프가 실행되지 않았습니다." << std::endl;
            return false;
        }
        
        std::lock_guard<std::mutex> lock(controlMutex_);
        targetAngle_ = normalizeAngle(currentAngle_ - 90.0);
        isTurning_ = true;
        turnDirection_ = -1; // 우회전
        
        std::cout << "우회전 90도 시작: " << currentAngle_ << "도 → " << targetAngle_ << "도" << std::endl;
        return true;
    }

    // 정지
    void stopTurning() {
        std::lock_guard<std::mutex> lock(controlMutex_);
        isTurning_ = false;
        turnDirection_ = 0;
        if (motorController_) {
            motorController_->stop();
        }
        std::cout << "회전 정지" << std::endl;
    }

    // 현재 상태 출력
    void printStatus() const {
        std::cout << "=== IMU-모터 통합 시스템 상태 ===" << std::endl;
        std::cout << "실행 상태: " << (isRunning_ ? "실행 중" : "정지") << std::endl;
        std::cout << "현재 각도: " << currentAngle_ << "도" << std::endl;
        std::cout << "목표 각도: " << targetAngle_ << "도" << std::endl;
        std::cout << "회전 상태: " << (isTurning_ ? "회전 중" : "정지") << std::endl;
        std::cout << "회전 방향: " << turnDirection_ << std::endl;
        std::cout << "각도 오차: " << (targetAngle_ - currentAngle_) << "도" << std::endl;
        std::cout << "================================" << std::endl;
    }

private:
    std::string i2cDevice_;
    std::shared_ptr<MyI2CImplementation> i2cInterface_;
    std::shared_ptr<IMUSensor> imuSensor_;
    std::shared_ptr<MotorDriver> motorDriver_;
    std::shared_ptr<MotorController> motorController_;
    
    std::atomic<bool> isRunning_;
    std::atomic<double> targetAngle_;
    std::atomic<double> currentAngle_;
    std::atomic<bool> isTurning_;
    std::atomic<int> turnDirection_;
    
    std::thread controlThread_;
    mutable std::mutex controlMutex_;
    
    // PID 제어 변수
    double pidKp_ = 2.0;
    double pidKi_ = 0.1;
    double pidKd_ = 0.5;
    double pidIntegral_ = 0.0;
    double pidPreviousError_ = 0.0;
    std::chrono::steady_clock::time_point lastPIDTime_;
    
    // 제어 루프
    void controlLoop() {
        const double angleTolerance = 2.0; // 각도 허용 오차 (도)
        const double maxSpeed = 50.0;      // 최대 속도
        const double minSpeed = 10.0;      // 최소 속도
        
        lastPIDTime_ = std::chrono::steady_clock::now();
        
        while (isRunning_) {
            auto startTime = std::chrono::steady_clock::now();
            
            // IMU 데이터 읽기
            if (imuSensor_->readData()) {
                auto data = imuSensor_->getLatestData();
                currentAngle_ = data.yaw;
                
                // 회전 제어
                if (isTurning_) {
                    double angleError = calculateAngleError(currentAngle_, targetAngle_);
                    
                    // 목표 각도에 도달했는지 확인
                    if (std::abs(angleError) <= angleTolerance) {
                        std::cout << "목표 각도 도달: " << currentAngle_ << "도 (오차: " << angleError << "도)" << std::endl;
                        stopTurning();
                    } else {
                        // PID 제어로 모터 속도 계산
                        double motorSpeed = calculatePIDOutput(angleError, 0.01); // 10ms = 0.01초
                        
                        // 속도 범위 제한
                        if (motorSpeed > maxSpeed) motorSpeed = maxSpeed;
                        else if (motorSpeed < -maxSpeed) motorSpeed = -maxSpeed;
                        
                        // 최소 속도 보장
                        if (std::abs(motorSpeed) < minSpeed && std::abs(angleError) > angleTolerance) {
                            motorSpeed = (motorSpeed >= 0) ? minSpeed : -minSpeed;
                        }
                        
                        // 모터 제어
                        if (motorController_) {
                            if (turnDirection_ == 1) { // 좌회전
                                motorController_->rotateLeft(static_cast<int>(std::abs(motorSpeed)));
                            } else if (turnDirection_ == -1) { // 우회전
                                motorController_->rotateRight(static_cast<int>(std::abs(motorSpeed)));
                            }
                        }
                        
                        // 디버그 출력 (1초마다)
                        static int debugCounter = 0;
                        if (++debugCounter >= 100) { // 100 * 10ms = 1초
                            std::cout << "회전 중 - 현재: " << currentAngle_ 
                                      << "도, 목표: " << targetAngle_ 
                                      << "도, 오차: " << angleError 
                                      << "도, 속도: " << motorSpeed << std::endl;
                            debugCounter = 0;
                        }
                    }
                }
            }
            
            // 10ms 대기 (100Hz 제어 주파수)
            auto endTime = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
            if (elapsed.count() < 10) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10 - elapsed.count()));
            }
        }
    }

    // PID 제어 출력 계산
    double calculatePIDOutput(double error, double dt) {
        // P 제어
        double pOutput = pidKp_ * error;
        
        // I 제어
        pidIntegral_ += error * dt;
        double iOutput = pidKi_ * pidIntegral_;
        
        // D 제어
        double derivative = (error - pidPreviousError_) / dt;
        double dOutput = pidKd_ * derivative;
        
        // 출력 계산
        double output = pOutput + iOutput + dOutput;
        
        // 상태 업데이트
        pidPreviousError_ = error;
        
        return output;
    }

    // 각도 정규화 (-180 ~ 180도)
    double normalizeAngle(double angle) {
        while (angle > 180.0) angle -= 360.0;
        while (angle < -180.0) angle += 360.0;
        return angle;
    }

    // 각도 오차 계산 (최단 경로)
    double calculateAngleError(double current, double target) {
        double error = target - current;
        
        // 최단 경로 계산
        if (error > 180.0) error -= 360.0;
        else if (error < -180.0) error += 360.0;
        
        return error;
    }
};

} // namespace amr
