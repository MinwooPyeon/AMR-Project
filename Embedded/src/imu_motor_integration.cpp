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
        , turnDirection_(0)
    {
        std::cout << "IMU-Motor integration system initialization started" << std::endl;
    }

    ~IMUMotorIntegration() {
        stop();
    }

    bool initialize() {
        try {

            i2cInterface_ = std::make_shared<MyI2CImplementation>(i2cDevice_);
            

            imuSensor_ = std::make_shared<IMUSensor>(i2cInterface_, IMUType::BNO08X, 0x4B, "BNO08x");
            
            if (!imuSensor_->initialize()) {
                std::cerr << "IMU sensor initialization failed" << std::endl;
                return false;
            }
            

            imuSensor_->setSampleRate(100);  // 100Hz 샘플링
            imuSensor_->setGyroRange(250);   // ±250°/s
            imuSensor_->setAccelRange(2);    // ±2g
            imuSensor_->enableLowPassFilter(true);
            imuSensor_->setFilterCutoff(5.0f);
            

            motorDriver_ = std::make_shared<MotorDriver>(i2cInterface_, 0x60, "MotorDriver");
            if (!motorDriver_->initialize()) {
                std::cerr << "Motor driver initialization failed" << std::endl;
                return false;
            }
            
            motorController_ = std::make_shared<MotorController>(motorDriver_, motorDriver_, "IMUMotorController");
            

            if (!calibrateInitialAngle()) {
                std::cerr << "Initial angle calibration failed" << std::endl;
                return false;
            }
            
            std::cout << "IMU-Motor integration system initialization completed" << std::endl;
            return true;
            
        } catch (const std::exception& e) {
            std::cerr << "Error during initialization: " << e.what() << std::endl;
            return false;
        }
    }

    bool calibrateInitialAngle() {
        std::cout << "Initial angle calibration started..." << std::endl;
        

        double sumAngle = 0.0;
        int sampleCount = 0;
        const int targetSamples = 300;
        
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
            std::cout << "Initial angle calibration completed: " << currentAngle_ << " degrees" << std::endl;
            return true;
        }
        
        return false;
    }

    void start() {
        if (isRunning_) {
            std::cout << "Already running." << std::endl;
            return;
        }
        
        isRunning_ = true;
        controlThread_ = std::thread(&IMUMotorIntegration::controlLoop, this);
        std::cout << "IMU-Motor integration control loop started" << std::endl;
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
        
        std::cout << "IMU-Motor integration control loop stopped" << std::endl;
    }


    bool turnLeft90() {
        if (!isRunning_) {
            std::cerr << "Control loop is not running." << std::endl;
            return false;
        }
        
        std::lock_guard<std::mutex> lock(controlMutex_);
        targetAngle_ = normalizeAngle(currentAngle_ + 90.0);
        isTurning_ = true;
        turnDirection_ = 1;
        
        std::cout << "Left turn 90 degrees started: " << currentAngle_ << " degrees → " << targetAngle_ << " degrees" << std::endl;
        return true;
    }


    bool turnRight90() {
        if (!isRunning_) {
            std::cerr << "Control loop is not running." << std::endl;
            return false;
        }
        
        std::lock_guard<std::mutex> lock(controlMutex_);
        targetAngle_ = normalizeAngle(currentAngle_ - 90.0);
        isTurning_ = true;
        turnDirection_ = -1;
        
        std::cout << "Right turn 90 degrees started: " << currentAngle_ << " degrees → " << targetAngle_ << " degrees" << std::endl;
        return true;
    }


    void stopTurning() {
        std::lock_guard<std::mutex> lock(controlMutex_);
        isTurning_ = false;
        turnDirection_ = 0;
        if (motorController_) {
            motorController_->stop();
        }
        std::cout << "Rotation stopped" << std::endl;
    }


    void printStatus() const {
            std::cout << "=== IMU-Motor Integration System Status ===" << std::endl;
    std::cout << "Running status: " << (isRunning_ ? "Running" : "Stopped") << std::endl;
            std::cout << "Current angle: " << currentAngle_ << " degrees" << std::endl;
    std::cout << "Target angle: " << targetAngle_ << " degrees" << std::endl;
            std::cout << "Turning status: " << (isTurning_ ? "Turning" : "Stopped") << std::endl;
    std::cout << "Turn direction: " << turnDirection_ << std::endl;
        std::cout << "Angle error: " << (targetAngle_ - currentAngle_) << " degrees" << std::endl;
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
    

    double pidKp_ = 2.0;
    double pidKi_ = 0.1;
    double pidKd_ = 0.5;
    double pidIntegral_ = 0.0;
    double pidPreviousError_ = 0.0;
    std::chrono::steady_clock::time_point lastPIDTime_;
    

    void controlLoop() {
        const double angleTolerance = 2.0;
        const double maxSpeed = 50.0;
        const double minSpeed = 10.0;
        
        lastPIDTime_ = std::chrono::steady_clock::now();
        
        while (isRunning_) {
            auto startTime = std::chrono::steady_clock::now();
            

            if (imuSensor_->readData()) {
                auto data = imuSensor_->getLatestData();
                currentAngle_ = data.yaw;
                

                if (isTurning_) {
                    double angleError = calculateAngleError(currentAngle_, targetAngle_);
                    

                    if (std::abs(angleError) <= angleTolerance) {
                        std::cout << "Target angle reached: " << currentAngle_ << " degrees (error: " << angleError << " degrees)" << std::endl;
                        stopTurning();
                    } else {

                        double motorSpeed = calculatePIDOutput(angleError, 0.01);
                        

                        if (motorSpeed > maxSpeed) motorSpeed = maxSpeed;
                        else if (motorSpeed < -maxSpeed) motorSpeed = -maxSpeed;
                        

                        if (std::abs(motorSpeed) < minSpeed && std::abs(angleError) > angleTolerance) {
                            motorSpeed = (motorSpeed >= 0) ? minSpeed : -minSpeed;
                        }
                        

                        if (motorController_) {
                            if (turnDirection_ == 1) {
                                motorController_->rotateLeft(static_cast<int>(std::abs(motorSpeed)));
                            } else if (turnDirection_ == -1) {
                                motorController_->rotateRight(static_cast<int>(std::abs(motorSpeed)));
                            }
                        }
                        

                        static int debugCounter = 0;
                        if (++debugCounter >= 100) {
                            std::cout << "Rotating - Current: " << currentAngle_ 
                                      << " degrees, Target: " << targetAngle_ 
                                      << " degrees, Error: " << angleError 
                                      << " degrees, Speed: " << motorSpeed << std::endl;
                            debugCounter = 0;
                        }
                    }
                }
            }
            

            auto endTime = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
            if (elapsed.count() < 10) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10 - elapsed.count()));
            }
        }
    }


    double calculatePIDOutput(double error, double dt) {

        double pOutput = pidKp_ * error;
        

        pidIntegral_ += error * dt;
        double iOutput = pidKi_ * pidIntegral_;
        

        double derivative = (error - pidPreviousError_) / dt;
        double dOutput = pidKd_ * derivative;
        

        double output = pOutput + iOutput + dOutput;
        

        pidPreviousError_ = error;
        
        return output;
    }


    double normalizeAngle(double angle) {
        while (angle > 180.0) angle -= 360.0;
        while (angle < -180.0) angle += 360.0;
        return angle;
    }


    double calculateAngleError(double current, double target) {
        double error = target - current;
        

        if (error > 180.0) error -= 360.0;
        else if (error < -180.0) error += 360.0;
        
        return error;
    }
};

} // namespace amr
