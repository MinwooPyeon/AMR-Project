#pragma once
#include <memory>
#include <string>
#include <atomic>
#include <mutex>
#include <thread>

namespace amr {

// 전방 선언
class IMUSensor;
class MotorController;
class MotorDriver;
class I2CInterface;

class IMUMotorIntegration {
public:
    IMUMotorIntegration(const std::string& i2cDevice = "/dev/i2c-1");
    ~IMUMotorIntegration();

    // 초기화 및 설정
    bool initialize();
    bool calibrateInitialAngle();

    // 제어 루프
    void start();
    void stop();

    // 회전 명령
    bool turnLeft90();      // 좌회전 90도
    bool turnRight90();     // 우회전 90도
    void stopTurning();     // 회전 정지

    // 상태 확인
    void printStatus() const;
    bool isRunning() const { return isRunning_; }
    bool isTurning() const { return isTurning_; }
    double getCurrentAngle() const { return currentAngle_; }
    double getTargetAngle() const { return targetAngle_; }

    // PID 설정
    void setPIDGains(double kp, double ki, double kd) {
        pidKp_ = kp;
        pidKi_ = ki;
        pidKd_ = kd;
    }

private:
    std::string i2cDevice_;
    std::shared_ptr<I2CInterface> i2cInterface_;
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
    
    // 내부 함수
    void controlLoop();
    double calculatePIDOutput(double error, double dt);
    double normalizeAngle(double angle);
    double calculateAngleError(double current, double target);
};

} // namespace amr
