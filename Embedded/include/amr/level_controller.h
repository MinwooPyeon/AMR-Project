#ifndef AMR_LEVEL_CONTROLLER_H
#define AMR_LEVEL_CONTROLLER_H

#include <memory>
#include <mutex>
#include <functional>
#include "imu_sensor.h"
#include "module/motor_controller.h"

namespace amr {

enum class LevelStatus {
    LEVEL = 0,
    TILTED_LEFT = 1,
    TILTED_RIGHT = 2,
    TILTED_FORWARD = 3,
    TILTED_BACKWARD = 4,
    UNKNOWN = 5
};

struct LevelControlConfig {
    float roll_threshold = 2.0f;
    float pitch_threshold = 2.0f;
    float correction_strength = 0.5f;
    int update_rate = 50;
    bool enable_auto_correction = true;
    bool enable_smoothing = true;
    float smoothing_factor = 0.1f;
};

struct LevelData {
    float roll;
    float pitch;
    float yaw;
    float roll_rate;
    float pitch_rate;
    float yaw_rate;
    LevelStatus status;
    uint64_t timestamp;
};

class LevelController {
public:
    LevelController(std::shared_ptr<IMUSensor> imu,
                   std::shared_ptr<MotorController> motor,
                   const LevelControlConfig& config = LevelControlConfig{});
    ~LevelController();

    bool initialize();
    void setConfig(const LevelControlConfig& config);
    LevelControlConfig getConfig() const;

    bool enableLevelControl(bool enable);
    bool isLevelControlEnabled() const;
    bool correctLevel();
    bool maintainLevel();

    LevelData getCurrentLevel() const;
    LevelStatus getLevelStatus() const;
    bool isLevel() const;
    float getRollAngle() const;
    float getPitchAngle() const;
    float getYawAngle() const;

    bool calibrateLevel();
    bool setLevelReference(float roll, float pitch);

    void setLevelCallback(std::function<void(const LevelData&)> callback);
    void setStatusCallback(std::function<void(LevelStatus)> callback);

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

    void updateLevelData();
    LevelStatus calculateLevelStatus(float roll, float pitch) const;
    void applySmoothing(LevelData& data);
    bool calculateCorrection(float roll_error, float pitch_error, 
                           int& left_speed, int& right_speed);
    void notifyCallbacks(const LevelData& data, LevelStatus status);
    
    float calculateRollCorrection(float roll_error) const;
    float calculatePitchCorrection(float pitch_error) const;
    void applyMotorCorrection(int left_speed, int right_speed);
};

} 

#endif