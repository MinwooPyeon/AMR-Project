#ifndef AMR_IMU_SENSOR_H
#define AMR_IMU_SENSOR_H

#include <memory>
#include <mutex>
#include <atomic>
#include <string>
#include "amr/i2c/i2c_interface.h"

namespace amr {

enum class IMUType {
    MPU6050,
    MPU9250,
    BNO055,
    LSM9DS1,
    BNO08X
};

struct IMUData {
    float gyro_x;
    float gyro_y;
    float gyro_z;
    
    float accel_x;
    float accel_y;
    float accel_z;
    
    float mag_x;
    float mag_y;
    float mag_z;
    
    float roll;
    float pitch;
    float yaw;
    
    float quaternion_w;
    float quaternion_x;
    float quaternion_y;
    float quaternion_z;
    
    float temperature;
    
    uint64_t timestamp;
};

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

    bool initialize();
    bool calibrate();
    bool reset();
    
    bool readData();
    IMUData getLatestData() const;
    
    IMUStatus getStatus() const;
    bool isConnected() const;
    bool isCalibrated() const;
    
    bool setSampleRate(uint16_t rate);
    bool setGyroRange(uint16_t range);
    bool setAccelRange(uint8_t range);
    
    void enableLowPassFilter(bool enable);
    void setFilterCutoff(float cutoff);
    
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
    
    uint16_t sampleRate_;
    uint16_t gyroRange_;
    uint8_t accelRange_;
    bool lowPassFilterEnabled_;
    float filterCutoff_;
    
    bool initializeMPU6050();
    bool initializeMPU9250();
    bool initializeAK8963();
    bool initializeBNO055();
    bool initializeLSM9DS1();
    bool initializeBNO08X();
    
    bool readMPU6050Data();
    bool readMPU9250Data();
    bool readAK8963Data();
    bool readBNO055Data();
    bool readLSM9DS1Data();
    bool readBNO08XData();
    
    bool writeRegister(uint8_t reg, uint8_t value);
    bool writeRegister(uint8_t addr, uint8_t reg, uint8_t value);
    uint8_t readRegister(uint8_t reg);
    uint8_t readRegister(uint8_t addr, uint8_t reg);
    bool testConnection();
    void updateStatus(IMUStatus newStatus);
    
    void calibrationTask();
    std::vector<IMUData> calibrationData_;
    
    float magCalibration_[3] = {1.0f, 1.0f, 1.0f};
};

} 

#endif 