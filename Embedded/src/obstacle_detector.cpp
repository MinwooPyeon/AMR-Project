#include "amr/obstacle_detector.h"
#include <iostream>
#include <algorithm>
#include <numeric>
#include <cmath>
#include <chrono>

namespace amr {

ObstacleDetector::ObstacleDetector(std::shared_ptr<IMUSensor> imu,
                                   const ObstacleDetectionConfig& config)
    : imu_(std::move(imu))
    , config_(config)
{
    if (!imu_) {
        std::cerr << "ObstacleDetector: IMU 센서가 제공되지 않았습니다." << std::endl;
        return;
    }
    
    std::cout << "ObstacleDetector: 장애물 감지기 초기화 시작" << std::endl;
}

ObstacleDetector::~ObstacleDetector() {
    std::cout << "ObstacleDetector: 장애물 감지기 종료" << std::endl;
}

bool ObstacleDetector::initialize() {
    if (!imu_) {
        std::cerr << "ObstacleDetector: IMU 센서가 없습니다." << std::endl;
        return false;
    }
    
    try {
        // IMU 센서 초기화
        if (!imu_->initialize()) {
            std::cerr << "ObstacleDetector: IMU 센서 초기화 실패" << std::endl;
            return false;
        }
        
        // 데이터 히스토리 초기화
        dataHistory_.clear();
        
        // 초기 데이터 수집
        for (int i = 0; i < config_.window_size; i++) {
            if (imu_->readData()) {
                updateDataHistory(imu_->getLatestData());
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        
        initialized_ = true;
        dataValid_ = dataHistory_.size() >= config_.window_size;
        
        std::cout << "ObstacleDetector: 초기화 완료 (데이터 포인트: " 
                  << dataHistory_.size() << ")" << std::endl;
        return true;
        
    } catch (const std::exception& e) {
        std::cerr << "ObstacleDetector: 초기화 오류: " << e.what() << std::endl;
        return false;
    }
}

void ObstacleDetector::setConfig(const ObstacleDetectionConfig& config) {
    std::lock_guard<std::mutex> lock(dataMutex_);
    config_ = config;
    std::cout << "ObstacleDetector: 설정 업데이트됨" << std::endl;
}

ObstacleDetectionConfig ObstacleDetector::getConfig() const {
    std::lock_guard<std::mutex> lock(dataMutex_);
    return config_;
}

ObstacleDetectionResult ObstacleDetector::detectObstacle() {
    if (!initialized_ || !dataValid_) {
        return ObstacleDetectionResult{false, 0.0f, 0.0f, 0.0f, 0, 0.0f, 0.0f, 0.0f};
    }
    
    try {
        // 최신 데이터 업데이트
        updateData();
        
        // 데이터 처리
        processData();
        
        // 장애물 감지 로직
        bool acceleration_anomaly = checkAccelerationAnomaly();
        bool gyro_anomaly = checkGyroAnomaly();
        bool magnetic_anomaly = config_.enable_magnetic_detection ? checkMagneticAnomaly() : false;
        
        // 결과 계산
        ObstacleDetectionResult result;
        result.obstacle_detected = acceleration_anomaly || gyro_anomaly || magnetic_anomaly;
        result.timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        
        if (result.obstacle_detected) {
            // 거리 추정
            result.distance_estimate = calculateDistance();
            result.impact_force = calculateImpactForce();
            
            // 신뢰도 계산
            float confidence = 0.0f;
            if (acceleration_anomaly) confidence += 0.4f;
            if (gyro_anomaly) confidence += 0.3f;
            if (magnetic_anomaly) confidence += 0.3f;
            result.confidence = std::min(confidence, 1.0f);
            
            // 방향 계산 (가속도 기반)
            if (!dataHistory_.empty()) {
                const auto& latest = dataHistory_.back();
                float magnitude = std::sqrt(latest.accel_x * latest.accel_x + 
                                         latest.accel_y * latest.accel_y + 
                                         latest.accel_z * latest.accel_z);
                if (magnitude > 0) {
                    result.direction_x = latest.accel_x / magnitude;
                    result.direction_y = latest.accel_y / magnitude;
                    result.direction_z = latest.accel_z / magnitude;
                }
            }
            
            // 콜백 호출
            if (obstacleCallback_) {
                obstacleCallback_(result);
            }
            if (distanceCallback_) {
                distanceCallback_(result.distance_estimate);
            }
        } else {
            result.distance_estimate = 0.0f;
            result.impact_force = 0.0f;
            result.confidence = 0.0f;
            result.direction_x = 0.0f;
            result.direction_y = 0.0f;
            result.direction_z = 0.0f;
        }
        
        lastResult_ = result;
        return result;
        
    } catch (const std::exception& e) {
        std::cerr << "ObstacleDetector: 장애물 감지 오류: " << e.what() << std::endl;
        return ObstacleDetectionResult{false, 0.0f, 0.0f, 0.0f, 0, 0.0f, 0.0f, 0.0f};
    }
}

bool ObstacleDetector::isObstacleDetected() const {
    return lastResult_.obstacle_detected;
}

void ObstacleDetector::updateData() {
    if (!imu_ || !initialized_) {
        return;
    }
    
    try {
        if (imu_->readData()) {
            IMUData data = imu_->getLatestData();
            
            // 필터링 적용
            if (filteringEnabled_) {
                applyLowPassFilter(data);
            }
            
            // 데이터 히스토리 업데이트
            updateDataHistory(data);
        }
        
    } catch (const std::exception& e) {
        std::cerr << "ObstacleDetector: 데이터 업데이트 오류: " << e.what() << std::endl;
    }
}

float ObstacleDetector::calculateDistance() {
    if (dataHistory_.empty()) {
        return 0.0f;
    }
    
    // 여러 방법으로 거리 추정
    float distance_accel = estimateDistanceFromAcceleration();
    float distance_mag = estimateDistanceFromMagneticField();
    float distance_motion = estimateDistanceFromMotion();
    
    // 가중 평균으로 최종 거리 계산
    float total_weight = 0.0f;
    float weighted_sum = 0.0f;
    
    if (distance_accel > 0) {
        weighted_sum += distance_accel * 0.5f;
        total_weight += 0.5f;
    }
    if (distance_mag > 0) {
        weighted_sum += distance_mag * 0.3f;
        total_weight += 0.3f;
    }
    if (distance_motion > 0) {
        weighted_sum += distance_motion * 0.2f;
        total_weight += 0.2f;
    }
    
    return total_weight > 0 ? weighted_sum / total_weight : 0.0f;
}

float ObstacleDetector::calculateImpactForce() {
    float force_accel = calculateImpactForceFromAcceleration();
    float force_gyro = calculateImpactForceFromGyro();
    
    // 가속도 기반 충격력이 더 신뢰할 수 있음
    return force_accel * 0.7f + force_gyro * 0.3f;
}

void ObstacleDetector::enableFiltering(bool enable) {
    filteringEnabled_ = enable;
}

void ObstacleDetector::setFilterStrength(float strength) {
    filterStrength_ = std::clamp(strength, 0.0f, 1.0f);
}

void ObstacleDetector::setObstacleCallback(std::function<void(const ObstacleDetectionResult&)> callback) {
    obstacleCallback_ = callback;
}

void ObstacleDetector::setDistanceCallback(std::function<void(float)> callback) {
    distanceCallback_ = callback;
}

bool ObstacleDetector::isInitialized() const {
    return initialized_;
}

bool ObstacleDetector::isDataValid() const {
    return dataValid_;
}

float ObstacleDetector::getLastDistance() const {
    return lastResult_.distance_estimate;
}

float ObstacleDetector::getLastConfidence() const {
    return lastResult_.confidence;
}

void ObstacleDetector::processData() {
    if (dataHistory_.size() < config_.window_size) {
        return;
    }
    
    // 데이터 유효성 확인
    dataValid_ = true;
}

bool ObstacleDetector::checkAccelerationAnomaly() {
    if (dataHistory_.size() < config_.window_size) {
        return false;
    }
    
    std::vector<float> accel_magnitudes;
    for (const auto& data : dataHistory_) {
        float magnitude = std::sqrt(data.accel_x * data.accel_x + 
                                  data.accel_y * data.accel_y + 
                                  data.accel_z * data.accel_z);
        accel_magnitudes.push_back(magnitude);
    }
    
    float current_magnitude = accel_magnitudes.back();
    float avg_magnitude = calculateMovingAverage(accel_magnitudes);
    float std_dev = calculateStandardDeviation(accel_magnitudes);
    
    // 임계값을 초과하는지 확인
    return std::abs(current_magnitude - avg_magnitude) > config_.acceleration_threshold;
}

bool ObstacleDetector::checkGyroAnomaly() {
    if (dataHistory_.size() < config_.window_size) {
        return false;
    }
    
    std::vector<float> gyro_magnitudes;
    for (const auto& data : dataHistory_) {
        float magnitude = std::sqrt(data.gyro_x * data.gyro_x + 
                                  data.gyro_y * data.gyro_y + 
                                  data.gyro_z * data.gyro_z);
        gyro_magnitudes.push_back(magnitude);
    }
    
    float current_magnitude = gyro_magnitudes.back();
    float avg_magnitude = calculateMovingAverage(gyro_magnitudes);
    
    return std::abs(current_magnitude - avg_magnitude) > config_.gyro_threshold;
}

bool ObstacleDetector::checkMagneticAnomaly() {
    if (dataHistory_.size() < config_.window_size) {
        return false;
    }
    
    std::vector<float> mag_magnitudes;
    for (const auto& data : dataHistory_) {
        float magnitude = std::sqrt(data.mag_x * data.mag_x + 
                                  data.mag_y * data.mag_y + 
                                  data.mag_z * data.mag_z);
        mag_magnitudes.push_back(magnitude);
    }
    
    float current_magnitude = mag_magnitudes.back();
    float avg_magnitude = calculateMovingAverage(mag_magnitudes);
    
    return std::abs(current_magnitude - avg_magnitude) > config_.magnetic_threshold;
}

float ObstacleDetector::calculateMovingAverage(const std::vector<float>& values) {
    if (values.empty()) return 0.0f;
    
    size_t start = values.size() > config_.window_size ? 
                   values.size() - config_.window_size : 0;
    
    float sum = 0.0f;
    for (size_t i = start; i < values.size(); i++) {
        sum += values[i];
    }
    
    return sum / (values.size() - start);
}

float ObstacleDetector::calculateStandardDeviation(const std::vector<float>& values) {
    if (values.size() < 2) return 0.0f;
    
    float mean = calculateMovingAverage(values);
    float sum_squares = 0.0f;
    
    for (float value : values) {
        sum_squares += (value - mean) * (value - mean);
    }
    
    return std::sqrt(sum_squares / (values.size() - 1));
}

void ObstacleDetector::applyLowPassFilter(IMUData& data) {
    if (dataHistory_.empty()) {
        return;
    }
    
    const auto& prev = dataHistory_.back();
    float alpha = filterStrength_;
    
    data.accel_x = alpha * data.accel_x + (1.0f - alpha) * prev.accel_x;
    data.accel_y = alpha * data.accel_y + (1.0f - alpha) * prev.accel_y;
    data.accel_z = alpha * data.accel_z + (1.0f - alpha) * prev.accel_z;
    
    data.gyro_x = alpha * data.gyro_x + (1.0f - alpha) * prev.gyro_x;
    data.gyro_y = alpha * data.gyro_y + (1.0f - alpha) * prev.gyro_y;
    data.gyro_z = alpha * data.gyro_z + (1.0f - alpha) * prev.gyro_z;
    
    data.mag_x = alpha * data.mag_x + (1.0f - alpha) * prev.mag_x;
    data.mag_y = alpha * data.mag_y + (1.0f - alpha) * prev.mag_y;
    data.mag_z = alpha * data.mag_z + (1.0f - alpha) * prev.mag_z;
}

void ObstacleDetector::updateDataHistory(const IMUData& data) {
    std::lock_guard<std::mutex> lock(dataMutex_);
    
    dataHistory_.push_back(data);
    
    // 윈도우 크기 유지
    while (dataHistory_.size() > config_.window_size) {
        dataHistory_.pop_front();
    }
}

float ObstacleDetector::estimateDistanceFromAcceleration() {
    if (dataHistory_.size() < 2) return 0.0f;
    
    // 가속도 적분을 통한 거리 추정
    // 간단한 근사치 계산
    const auto& current = dataHistory_.back();
    const auto& previous = dataHistory_[dataHistory_.size() - 2];
    
    float dt = (current.timestamp - previous.timestamp) / 1000.0f; // 초 단위
    if (dt <= 0) return 0.0f;
    
    float accel_magnitude = std::sqrt(current.accel_x * current.accel_x + 
                                     current.accel_y * current.accel_y + 
                                     current.accel_z * current.accel_z);
    
    // 간단한 거리 추정 (가속도 기반)
    return accel_magnitude * dt * dt * 0.5f;
}

float ObstacleDetector::estimateDistanceFromMagneticField() {
    if (dataHistory_.size() < 2) return 0.0f;
    
    // 자기장 변화를 통한 거리 추정
    const auto& current = dataHistory_.back();
    const auto& previous = dataHistory_[dataHistory_.size() - 2];
    
    float mag_change = std::sqrt(
        (current.mag_x - previous.mag_x) * (current.mag_x - previous.mag_x) +
        (current.mag_y - previous.mag_y) * (current.mag_y - previous.mag_y) +
        (current.mag_z - previous.mag_z) * (current.mag_z - previous.mag_z)
    );
    
    // 자기장 변화를 거리로 변환 (근사치)
    return mag_change / 1000.0f; // μT를 미터로 변환
}

float ObstacleDetector::estimateDistanceFromMotion() {
    if (dataHistory_.size() < 2) return 0.0f;
    
    // 모션 패턴을 통한 거리 추정
    const auto& current = dataHistory_.back();
    
    float motion_magnitude = std::sqrt(
        current.gyro_x * current.gyro_x +
        current.gyro_y * current.gyro_y +
        current.gyro_z * current.gyro_z
    );
    
    // 각속도를 거리로 변환 (근사치)
    return motion_magnitude / 100.0f;
}

float ObstacleDetector::calculateImpactForceFromAcceleration() {
    if (dataHistory_.empty()) return 0.0f;
    
    const auto& data = dataHistory_.back();
    float accel_magnitude = std::sqrt(data.accel_x * data.accel_x + 
                                     data.accel_y * data.accel_y + 
                                     data.accel_z * data.accel_z);
    
    // 가속도를 힘으로 변환 (질량 = 1kg 가정)
    return accel_magnitude;
}

float ObstacleDetector::calculateImpactForceFromGyro() {
    if (dataHistory_.empty()) return 0.0f;
    
    const auto& data = dataHistory_.back();
    float gyro_magnitude = std::sqrt(data.gyro_x * data.gyro_x + 
                                    data.gyro_y * data.gyro_y + 
                                    data.gyro_z * data.gyro_z);
    
    // 각속도를 힘으로 변환 (근사치)
    return gyro_magnitude / 10.0f;
}

} // namespace amr 