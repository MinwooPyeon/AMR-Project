#pragma once
#include <memory>
#include <string>
#include <atomic>
#include <mutex>
#include <chrono>
#include <thread>
#include "amr/module/motor_controller.h"
#include "amr/imu_sensor.h"

namespace amr {

enum class AngleControlStatus {
    IDLE = 0,
    TURNING_LEFT = 1,
    TURNING_RIGHT = 2,
    TURNING_180 = 3,
    CALIBRATING = 4,
    ERROR = 5
};

enum class TurnType {
    LEFT_90 = 0,
    RIGHT_90 = 1,
    TURN_180 = 2
};

struct AngleControlConfig {
    double angleTolerance = 2.0;
    double turnSpeed = 50.0;
    double maxTurnTime = 10.0;
    double controlFrequency = 50.0;
    double pidKp = 2.0;
    double pidKi = 0.1;
    double pidKd = 0.5;
    bool enablePID = true;
    bool enableSmoothing = true;
    double smoothingFactor = 0.8;
};

struct AngleControlResult {
    bool success = false;
    AngleControlStatus status = AngleControlStatus::IDLE;
    double currentAngle = 0.0;
    double targetAngle = 0.0;
    double angleError = 0.0;
    double turnProgress = 0.0;
    std::string message = "";
    std::chrono::steady_clock::time_point timestamp;
};

class AngleController {
public:
    AngleController(std::shared_ptr<MotorController> motorController,
                   std::shared_ptr<IMUSensor> imuSensor,
                   const std::string& name = "AngleController");
    ~AngleController();

    bool initialize();
    bool calibrate();
    void setConfig(const AngleControlConfig& config);
    AngleControlConfig getConfig() const;

    bool turnLeft90();
    bool turnRight90();
    bool turn180();
    bool turnToAngle(double targetAngle);
    bool stop();

    AngleControlStatus getStatus() const;
    bool isActive() const;
    AngleControlResult getCurrentResult() const;
    double getCurrentAngle() const;
    double getTargetAngle() const;

    void startControlLoop();
    void stopControlLoop();
    bool isControlLoopRunning() const;

    void setPIDGains(double kp, double ki, double kd);
    void enablePID(bool enable);
    void resetPID();

    void enableSmoothing(bool enable);
    void setSmoothingFactor(double factor);

    bool testRotationControl();
    void printStatus() const;

private:
    std::shared_ptr<MotorController> motorController_;
    std::shared_ptr<IMUSensor> imuSensor_;
    std::string name_;
    
    mutable std::mutex mutex_;
    std::atomic<AngleControlStatus> status_{AngleControlStatus::IDLE};
    std::atomic<bool> controlLoopRunning_{false};
    std::atomic<double> currentAngle_{0.0};
    std::atomic<double> targetAngle_{0.0};
    std::atomic<double> angleError_{0.0};
    std::atomic<double> initialAngle_{0.0};  // 회전 시작 시 초기 각도
    
    AngleControlConfig config_;
    AngleControlResult currentResult_;
    
    double pidIntegral_ = 0.0;
    double pidPreviousError_ = 0.0;
    std::chrono::steady_clock::time_point lastPIDTime_;
    
    double smoothedAngle_ = 0.0;
    bool firstSmoothing_ = true;
    
    std::thread controlLoopThread_;
    std::atomic<bool> shouldStopLoop_{false};
    
    void controlLoop();
    double calculatePIDOutput(double error, double dt);
    double smoothAngle(double newAngle);
    double normalizeAngle(double angle);
    double calculateAngleError(double current, double target);
    void updateCurrentResult();
    void logControl(const std::string& message);
    
    void emergencyStop();
    bool checkSafetyConditions();
    
    double calculateTargetAngle(TurnType turnType);
    double calculateTurnProgress();
};

} 