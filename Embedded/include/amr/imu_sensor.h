#ifndef AMR_IMU_SENSOR_H
#define AMR_IMU_SENSOR_H

#include <memory>
#include <mutex>
#include <atomic>
#include <string>
#include "amr/i2c/i2c_interface.h"

namespace amr {

// IMU 센서 타입
enum class IMUType {
    MPU6050,
    MPU9250,
    BNO055,
    LSM9DS1
};

// IMU 데이터 구조체
struct IMUData {
    // 자이로스코프 (도/초)
    float gyro_x;
    float gyro_y;
    float gyro_z;
    
    // 가속도계 (m/s²)
    float accel_x;
    float accel_y;
    float accel_z;
    
    // 자력계 (μT) - 지원하는 경우
    float mag_x;
    float mag_y;
    float mag_z;
    
    // 오일러 각도 (도) - 지원하는 경우
    float roll;
    float pitch;
    float yaw;
    
    // 쿼터니언 - 지원하는 경우
    float quaternion_w;
    float quaternion_x;
    float quaternion_y;
    float quaternion_z;
    
    // 온도 (섭씨)
    float temperature;
    
    // 타임스탬프
    uint64_t timestamp;
};

// IMU 센서 상태
enum class IMUStatus {
    OK,
    ERROR_I2C,
    ERROR_CALIBRATION,
    ERROR_COMMUNICATION
};

class IMUSensor {
public:
    IMUSensor(std::shared_ptr<I2CInterface> i2c, 
              IMUType type = IMUType::MPU6050,
              uint8_t i2cAddr = 0x68,
              const std::string& name = "IMU");
    ~IMUSensor();

    // 초기화 및 설정
    bool initialize();
    bool calibrate();
    bool reset();
    
    // 데이터 읽기
    bool readData();
    IMUData getLatestData() const;
    
    // 상태 확인
    IMUStatus getStatus() const;
    bool isConnected() const;
    bool isCalibrated() const;
    
    // 설정
    bool setSampleRate(uint16_t rate);
    bool setGyroRange(uint16_t range);
    bool setAccelRange(uint8_t range);
    
    // 필터링 및 보정
    void enableLowPassFilter(bool enable);
    void setFilterCutoff(float cutoff);
    
    // 캘리브레이션
    bool startCalibration();
    bool stopCalibration();
    float getCalibrationProgress() const;

private:
    std::shared_ptr<I2CInterface> i2c_;
    IMUType type_;
    uint8_t i2cAddr_;
    std::string name_;
    
    mutable std::mutex dataMutex_;
    IMUData latestData_;
    std::atomic<IMUStatus> status_{IMUStatus::OK};
    std::atomic<bool> connected_{false};
    std::atomic<bool> calibrated_{false};
    std::atomic<bool> calibrating_{false};
    std::atomic<float> calibrationProgress_{0.0f};
    
    // 설정값
    uint16_t sampleRate_;
    uint16_t gyroRange_;
    uint8_t accelRange_;
    bool lowPassFilterEnabled_;
    float filterCutoff_;
    
    // 내부 함수
    bool initializeMPU6050();
    bool initializeMPU9250();
    bool initializeAK8963();
    bool initializeBNO055();
    bool initializeLSM9DS1();
    
    bool readMPU6050Data();
    bool readMPU9250Data();
    bool readAK8963Data();
    bool readBNO055Data();
    bool readLSM9DS1Data();
    
    bool writeRegister(uint8_t reg, uint8_t value);
    bool writeRegister(uint8_t addr, uint8_t reg, uint8_t value);
    uint8_t readRegister(uint8_t reg);
    uint8_t readRegister(uint8_t addr, uint8_t reg);
    bool testConnection();
    void updateStatus(IMUStatus newStatus);
    
    // 캘리브레이션 관련
    void calibrationTask();
    std::vector<IMUData> calibrationData_;
    
    // 자력계 캘리브레이션 값
    float magCalibration_[3] = {1.0f, 1.0f, 1.0f};
};

} // namespace amr

#endif // AMR_IMU_SENSOR_H 