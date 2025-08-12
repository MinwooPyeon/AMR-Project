#ifndef AMR_LEVEL_CONTROLLER_H
#define AMR_LEVEL_CONTROLLER_H

#include <memory>
#include <mutex>
#include <functional>
#include "imu_sensor.h"
#include "module/motor_controller.h"

namespace amr {

enum class LevelStatus {
    LEVEL = 0,           // 수평 상태
    TILTED_LEFT = 1,     // 왼쪽으로 기울어짐
    TILTED_RIGHT = 2,    // 오른쪽으로 기울어짐
    TILTED_FORWARD = 3,  // 앞으로 기울어짐
    TILTED_BACKWARD = 4, // 뒤로 기울어짐
    UNKNOWN = 5          // 알 수 없음
};

struct LevelControlConfig {
    float roll_threshold = 2.0f;      // 롤 각도 임계값 (도)
    float pitch_threshold = 2.0f;     // 피치 각도 임계값 (도)
    float correction_strength = 0.5f; // 보정 강도 (0.0 ~ 1.0)
    int update_rate = 50;             // 업데이트 주기 (Hz)
    bool enable_auto_correction = true; // 자동 보정 활성화
    bool enable_smoothing = true;      // 스무딩 필터 활성화
    float smoothing_factor = 0.1f;     // 스무딩 계수
};

struct LevelData {
    float roll;          // 롤 각도 (도)
    float pitch;         // 피치 각도 (도)
    float yaw;           // 요 각도 (도)
    float roll_rate;     // 롤 각속도 (도/초)
    float pitch_rate;    // 피치 각속도 (도/초)
    float yaw_rate;      // 요 각속도 (도/초)
    LevelStatus status;  // 수평 상태
    uint64_t timestamp;  // 타임스탬프
};

class LevelController {
public:
    LevelController(std::shared_ptr<IMUSensor> imu,
                   std::shared_ptr<MotorController> motor,
                   const LevelControlConfig& config = LevelControlConfig{});
    ~LevelController();

    // 초기화 및 설정
    bool initialize();
    void setConfig(const LevelControlConfig& config);
    LevelControlConfig getConfig() const;

    // 수평 유지 제어
    bool enableLevelControl(bool enable);
    bool isLevelControlEnabled() const;
    bool correctLevel();
    bool maintainLevel();

    // 데이터 읽기
    LevelData getCurrentLevel() const;
    LevelStatus getLevelStatus() const;
    bool isLevel() const;
    float getRollAngle() const;
    float getPitchAngle() const;
    float getYawAngle() const;

    // 캘리브레이션
    bool calibrateLevel();
    bool setLevelReference(float roll, float pitch);

    // 콜백 설정
    void setLevelCallback(std::function<void(const LevelData&)> callback);
    void setStatusCallback(std::function<void(LevelStatus)> callback);

    // 필터링
    void enableSmoothing(bool enable);
    void setSmoothingFactor(float factor);

private:
    std::shared_ptr<IMUSensor> imu_;
    std::shared_ptr<MotorController> motor_;
    LevelControlConfig config_;
    
    mutable std::mutex dataMutex_;
    LevelData currentLevel_;
    LevelData referenceLevel_;
    
    bool initialized_;
    bool levelControlEnabled_;
    bool smoothingEnabled_;
    
    std::function<void(const LevelData&)> levelCallback_;
    std::function<void(LevelStatus)> statusCallback_;

    // 내부 메서드
    void updateLevelData();
    LevelStatus calculateLevelStatus(float roll, float pitch) const;
    void applySmoothing(LevelData& data);
    bool calculateCorrection(float roll_error, float pitch_error, 
                           int& left_speed, int& right_speed);
    void notifyCallbacks(const LevelData& data, LevelStatus status);
    
    // 수평 보정 알고리즘
    float calculateRollCorrection(float roll_error) const;
    float calculatePitchCorrection(float pitch_error) const;
    void applyMotorCorrection(int left_speed, int right_speed);
};

} // namespace amr

#endif // AMR_LEVEL_CONTROLLER_H 