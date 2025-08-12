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

// 장애물 감지 결과
struct ObstacleDetectionResult {
    bool obstacle_detected;
    float distance_estimate;  // 추정 거리 (미터)
    float confidence;         // 신뢰도 (0.0 ~ 1.0)
    float impact_force;       // 충격력 추정
    uint64_t timestamp;
    
    // 장애물 방향 정보
    float direction_x;
    float direction_y;
    float direction_z;
};

// 장애물 감지 설정
struct ObstacleDetectionConfig {
    float acceleration_threshold = 2.0f;    // 가속도 임계값 (m/s²)
    float gyro_threshold = 50.0f;          // 자이로 임계값 (도/초)
    float magnetic_threshold = 100.0f;      // 자기장 임계값 (μT)
    int window_size = 10;                   // 이동 평균 윈도우 크기
    float detection_confidence = 0.7f;      // 감지 신뢰도 임계값
    bool enable_magnetic_detection = true;  // 자기장 기반 감지 활성화
    bool enable_motion_detection = true;    // 모션 기반 감지 활성화
};

class ObstacleDetector {
public:
    ObstacleDetector(std::shared_ptr<IMUSensor> imu,
                    const ObstacleDetectionConfig& config = ObstacleDetectionConfig{});
    ~ObstacleDetector();

    // 초기화 및 설정
    bool initialize();
    void setConfig(const ObstacleDetectionConfig& config);
    ObstacleDetectionConfig getConfig() const;
    
    // 장애물 감지
    ObstacleDetectionResult detectObstacle();
    bool isObstacleDetected() const;
    
    // 데이터 분석
    void updateData();
    float calculateDistance();
    float calculateImpactForce();
    
    // 필터링 및 보정
    void enableFiltering(bool enable);
    void setFilterStrength(float strength);
    
    // 콜백 설정
    void setObstacleCallback(std::function<void(const ObstacleDetectionResult&)> callback);
    void setDistanceCallback(std::function<void(float)> callback);
    
    // 상태 확인
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
    
    // 콜백 함수
    std::function<void(const ObstacleDetectionResult&)> obstacleCallback_;
    std::function<void(float)> distanceCallback_;
    
    // 내부 함수
    void processData();
    bool checkAccelerationAnomaly();
    bool checkGyroAnomaly();
    bool checkMagneticAnomaly();
    float calculateMovingAverage(const std::vector<float>& values);
    float calculateStandardDeviation(const std::vector<float>& values);
    void applyLowPassFilter(IMUData& data);
    void updateDataHistory(const IMUData& data);
    
    // 거리 추정 알고리즘
    float estimateDistanceFromAcceleration();
    float estimateDistanceFromMagneticField();
    float estimateDistanceFromMotion();
    
    // 충격력 계산
    float calculateImpactForceFromAcceleration();
    float calculateImpactForceFromGyro();
};

} // namespace amr

#endif // AMR_OBSTACLE_DETECTOR_H 