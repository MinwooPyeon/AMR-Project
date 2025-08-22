#ifndef AMR_OBSTACLE_DETECTOR_H
#define AMR_OBSTACLE_DETECTOR_H

#include <memory>
#include <vector>
#include <deque>
#include <mutex>
#include <atomic>
#include <functional>
#include "amr/imu_sensor.h"

namespace amr {

struct ObstacleDetectionResult {
    bool obstacle_detected;
    float distance_estimate;
    float confidence;
    float impact_force;
    uint64_t timestamp;
    
    float direction_x;
    float direction_y;
    float direction_z;
};

struct ObstacleDetectionConfig {
    float acceleration_threshold = 2.0f;
    float gyro_threshold = 50.0f;
    float magnetic_threshold = 100.0f;
    int window_size = 10;
    float detection_confidence = 0.7f;
    bool enable_magnetic_detection = true;
    bool enable_motion_detection = true;
};

class ObstacleDetector {
public:
    ObstacleDetector(std::shared_ptr<IMUSensor> imu,
                    const ObstacleDetectionConfig& config = ObstacleDetectionConfig{});
    ~ObstacleDetector();

    bool initialize();
    void setConfig(const ObstacleDetectionConfig& config);
    ObstacleDetectionConfig getConfig() const;
    
    ObstacleDetectionResult detectObstacle();
    bool isObstacleDetected() const;
    
    void updateData();
    float calculateDistance();
    float calculateImpactForce();
    
    void enableFiltering(bool enable);
    void setFilterStrength(float strength);
    
    void setObstacleCallback(std::function<void(const ObstacleDetectionResult&)> callback);
    void setDistanceCallback(std::function<void(float)> callback);
    
    bool isInitialized() const;
    bool isDataValid() const;
    float getLastDistance() const;
    float getLastConfidence() const;

private:
    std::shared_ptr<IMUSensor> imu_;
    ObstacleDetectionConfig config_;
    
    mutable std::mutex dataMutex_;
    std::deque<IMUData> dataHistory_;
    ObstacleDetectionResult lastResult_;
    
    std::atomic<bool> initialized_{false};
    std::atomic<bool> dataValid_{false};
    std::atomic<bool> filteringEnabled_{true};
    std::atomic<float> filterStrength_{0.1f};
    
    std::function<void(const ObstacleDetectionResult&)> obstacleCallback_;
    std::function<void(float)> distanceCallback_;
    
    void processData();
    bool checkAccelerationAnomaly();
    bool checkGyroAnomaly();
    bool checkMagneticAnomaly();
    float calculateMovingAverage(const std::vector<float>& values);
    float calculateStandardDeviation(const std::vector<float>& values);
    void applyLowPassFilter(IMUData& data);
    void updateDataHistory(const IMUData& data);
    
    float estimateDistanceFromAcceleration();
    float estimateDistanceFromMagneticField();
    float estimateDistanceFromMotion();
    
    float calculateImpactForceFromAcceleration();
    float calculateImpactForceFromGyro();
};

} 

#endif 