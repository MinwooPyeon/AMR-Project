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
    LEFT_90 = 0,      // 좌회전 90도
    RIGHT_90 = 1,     // 우회전 90도
    TURN_180 = 2      // 회전 180도
};

struct AngleControlConfig {
    double angleTolerance = 2.0;        // 각도 허용 오차 (도)
    double turnSpeed = 50.0;            // 회전 속도 (0-100)
    double maxTurnTime = 10.0;          // 최대 회전 시간 (초)
    double controlFrequency = 50.0;     // 제어 주파수 (Hz)
    double pidKp = 2.0;                // PID P 게인
    double pidKi = 0.1;                // PID I 게인
    double pidKd = 0.5;                // PID D 게인
    bool enablePID = true;              // PID 제어 활성화
    bool enableSmoothing = true;        // 각도 스무딩 활성화
    double smoothingFactor = 0.8;       // 스무딩 팩터 (0-1)
};

struct AngleControlResult {
    bool success = false;
    AngleControlStatus status = AngleControlStatus::IDLE;
    double currentAngle = 0.0;
    double targetAngle = 0.0;
    double angleError = 0.0;
    double turnProgress = 0.0;          // 회전 진행률 (0-1)
    std::string message = "";
    std::chrono::steady_clock::time_point timestamp;
};

class AngleController {
public:
    AngleController(std::shared_ptr<MotorController> motorController,
                   std::shared_ptr<IMUSensor> imuSensor,
                   const std::string& name = "AngleController");
    ~AngleController();

    // 초기화 및 설정
    bool initialize();
    bool calibrate();
    void setConfig(const AngleControlConfig& config);
    AngleControlConfig getConfig() const;

    // 회전 제어 명령
    bool turnLeft90();                  // 좌회전 90도
    bool turnRight90();                 // 우회전 90도
    bool turn180();                     // 회전 180도
    bool turnToAngle(double targetAngle); // 특정 각도로 회전
    bool stop();

    // 상태 확인
    AngleControlStatus getStatus() const;
    bool isActive() const;
    AngleControlResult getCurrentResult() const;
    double getCurrentAngle() const;
    double getTargetAngle() const;

    // 제어 루프
    void startControlLoop();
    void stopControlLoop();
    bool isControlLoopRunning() const;

    // PID 제어
    void setPIDGains(double kp, double ki, double kd);
    void enablePID(bool enable);
    void resetPID();

    // 스무딩 설정
    void enableSmoothing(bool enable);
    void setSmoothingFactor(double factor);

    // 진단 및 테스트
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
    
    // PID 제어 변수
    double pidIntegral_ = 0.0;
    double pidPreviousError_ = 0.0;
    std::chrono::steady_clock::time_point lastPIDTime_;
    
    // 스무딩 변수
    double smoothedAngle_ = 0.0;
    bool firstSmoothing_ = true;
    
    // 제어 루프
    std::thread controlLoopThread_;
    std::atomic<bool> shouldStopLoop_{false};
    
    // 내부 헬퍼 함수
    void controlLoop();
    double calculatePIDOutput(double error, double dt);
    double smoothAngle(double newAngle);
    double normalizeAngle(double angle);
    double calculateAngleError(double current, double target);
    void updateCurrentResult();
    void logControl(const std::string& message);
    
    // 안전 함수
    void emergencyStop();
    bool checkSafetyConditions();
    
    // 회전 계산 함수
    double calculateTargetAngle(TurnType turnType);
    double calculateTurnProgress();
};

} // namespace amr 