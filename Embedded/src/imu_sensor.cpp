#include "amr/imu_sensor.h"
#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>
#include <algorithm>

namespace amr {

namespace MPU6050Registers {
    const uint8_t WHO_AM_I = 0x75;
    const uint8_t PWR_MGMT_1 = 0x6B;
    const uint8_t PWR_MGMT_2 = 0x6C;
    const uint8_t CONFIG = 0x1A;
    const uint8_t GYRO_CONFIG = 0x1B;
    const uint8_t ACCEL_CONFIG = 0x1C;
    const uint8_t SMPLRT_DIV = 0x19;
    const uint8_t INT_ENABLE = 0x38;
    const uint8_t INT_STATUS = 0x3A;
    const uint8_t ACCEL_XOUT_H = 0x3B;
    const uint8_t GYRO_XOUT_H = 0x43;
    const uint8_t TEMP_OUT_H = 0x41;
    const uint8_t USER_CTRL = 0x6A;
    const uint8_t FIFO_EN = 0x23;
    const uint8_t INT_PIN_CFG = 0x37;
    const uint8_t I2C_MST_CTRL = 0x24;
}

namespace AK8963Registers {
    const uint8_t WHO_AM_I = 0x00;      
    const uint8_t INFO = 0x01;
    const uint8_t ST1 = 0x02;           
    const uint8_t XOUT_L = 0x03;        
    const uint8_t XOUT_H = 0x04;
    const uint8_t YOUT_L = 0x05;
    const uint8_t YOUT_H = 0x06;
    const uint8_t ZOUT_L = 0x07;
    const uint8_t ZOUT_H = 0x08;
    const uint8_t ST2 = 0x09;           
    const uint8_t CNTL = 0x0A;          
    const uint8_t ASTC = 0x0C;          
    const uint8_t I2CDIS = 0x0F;        
    const uint8_t ASAX = 0x10;          
    const uint8_t ASAY = 0x11;          
    const uint8_t ASAZ = 0x12;          
}

const uint8_t AK8963_ADDRESS = 0x0C;

IMUSensor::IMUSensor(std::shared_ptr<I2CInterface> i2c, 
                     IMUType type,
                     uint8_t i2cAddr,
                     const std::string& name)
    : i2c_(std::move(i2c))
    , type_(type)
    , i2cAddr_(i2cAddr)
    , name_(name)
    , sampleRate_(1000)
    , gyroRange_(250)
    , accelRange_(2)
    , lowPassFilterEnabled_(true)
    , filterCutoff_(5.0f)
{
    if (!i2c_) {
        updateStatus(IMUStatus::ERROR_I2C);
        return;
    }
    
    latestData_ = {};
    latestData_.timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    
    std::cout << name_ << ": IMU sensor initialization started (type: " << static_cast<int>(type_) 
              << ", address: 0x" << std::hex << (int)i2cAddr_ << std::dec << ")" << std::endl;
}

IMUSensor::~IMUSensor() {
    stopCalibration();
}

bool IMUSensor::initialize() {
    if (!i2c_) {
        updateStatus(IMUStatus::ERROR_I2C);
        return false;
    }
    
    try {
        if (!testConnection()) {
            updateStatus(IMUStatus::ERROR_COMMUNICATION);
            return false;
        }
        
        bool success = false;
        switch (type_) {
            case IMUType::MPU6050:
                success = initializeMPU6050();
                break;
            case IMUType::MPU9250:
                success = initializeMPU9250();
                break;
            case IMUType::BNO055:
                success = initializeBNO055();
                break;
            case IMUType::LSM9DS1:
                success = initializeLSM9DS1();
                break;
            case IMUType::BNO08X:
                success = initializeBNO08X();
                break;
        }
        
        if (success) {
            connected_ = true;
            updateStatus(IMUStatus::OK);
            std::cout << name_ << ": IMU sensor initialization completed" << std::endl;
            return true;
        } else {
            updateStatus(IMUStatus::ERROR_COMMUNICATION);
            return false;
        }
        
    } catch (const std::exception& e) {
        std::cerr << name_ << ": Initialization error: " << e.what() << std::endl;
        updateStatus(IMUStatus::ERROR_COMMUNICATION);
        return false;
    }
}

bool IMUSensor::initializeMPU6050() {
    try {
        uint8_t who_am_i = readRegister(MPU6050Registers::WHO_AM_I);
        if (who_am_i != 0x68) {
            std::cerr << name_ << "0x" 
                      << std::hex << (int)who_am_i << std::dec << ")" << std::endl;
            return false;
        }
        
        writeRegister(MPU6050Registers::PWR_MGMT_1, 0x80);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        writeRegister(MPU6050Registers::PWR_MGMT_1, 0x01);
        
        writeRegister(MPU6050Registers::SMPLRT_DIV, 0x07); 
        
        writeRegister(MPU6050Registers::CONFIG, 0x06); 
        
        uint8_t gyro_config = 0;
        switch (gyroRange_) {
            case 250: gyro_config = 0x00; break;
            case 500: gyro_config = 0x08; break;
            case 1000: gyro_config = 0x10; break;
            case 2000: gyro_config = 0x18; break;
            default: gyro_config = 0x00; break;
        }
        writeRegister(MPU6050Registers::GYRO_CONFIG, gyro_config);
        
        uint8_t accel_config = 0;
        switch (accelRange_) {
            case 2: accel_config = 0x00; break;
            case 4: accel_config = 0x08; break;
            case 8: accel_config = 0x10; break;
            case 16: accel_config = 0x18; break;
            default: accel_config = 0x00; break;
        }
        writeRegister(MPU6050Registers::ACCEL_CONFIG, accel_config);
        
        writeRegister(MPU6050Registers::INT_ENABLE, 0x01);
        
        std::cout << name_ << ": MPU6050 initialization completed" << std::endl;
        return true;
        
    } catch (const std::exception& e) {
        std::cerr << name_ << ": MPU6050 initialization error: " << e.what() << std::endl;
        return false;
    }
}

bool IMUSensor::initializeMPU9250() {
    try {
        if (!initializeMPU6050()) {
            return false;
        }
        
        writeRegister(MPU6050Registers::INT_PIN_CFG, 0x22);
        writeRegister(MPU6050Registers::INT_ENABLE, 0x01);
        
        if (!initializeAK8963()) {
            std::cerr << name_ << ": AK8963 magnetometer initialization failed" << std::endl;
            return false;
        }
        
        std::cout << name_ << ": MPU9250 initialization completed (with magnetometer)" << std::endl;
        return true;
        
    } catch (const std::exception& e) {
        std::cerr << name_ << ": MPU9250 initialization error: " << e.what() << std::endl;
        return false;
    }
}

bool IMUSensor::initializeAK8963() {
    try {
        uint8_t who_am_i = readRegister(AK8963_ADDRESS, AK8963Registers::WHO_AM_I);
        if (who_am_i != 0x48) {
            std::cerr << name_ << ": AK8963 WHO_AM_I 불일치 (예상: 0x48, 실제: 0x" 
                      << std::hex << (int)who_am_i << std::dec << ")" << std::endl;
            return false;
        }
        
        writeRegister(AK8963_ADDRESS, AK8963Registers::CNTL, 0x00);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        
        writeRegister(AK8963_ADDRESS, AK8963Registers::CNTL, 0x0F);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        
        uint8_t asa[3];
        for (int i = 0; i < 3; i++) {
            asa[i] = readRegister(AK8963_ADDRESS, AK8963Registers::ASAX + i);
        }
        
        magCalibration_[0] = (float)(asa[0] - 128) / 256.0f + 1.0f;
        magCalibration_[1] = (float)(asa[1] - 128) / 256.0f + 1.0f;
        magCalibration_[2] = (float)(asa[2] - 128) / 256.0f + 1.0f;
        
        writeRegister(AK8963_ADDRESS, AK8963Registers::CNTL, 0x00);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        
        writeRegister(AK8963_ADDRESS, AK8963Registers::CNTL, 0x16);
        
        std::cout << name_ << ": AK8963 magnetometer initialization completed" << std::endl;
        return true;
        
    } catch (const std::exception& e) {
        std::cerr << name_ << ": AK8963 initialization error: " << e.what() << std::endl;
        return false;
    }
}

bool IMUSensor::initializeBNO055() {
    std::cout << name_ << ": BNO055 initialization (to be implemented)" << std::endl;
    return false; 
}

bool IMUSensor::initializeLSM9DS1() {
    std::cout << name_ << ": LSM9DS1 initialization (to be implemented)" << std::endl;
    return false; 
}

bool IMUSensor::initializeBNO08X() {
    std::cout << name_ << ": BNO08x initialization (stub)" << std::endl;
    connected_ = true;
    updateStatus(IMUStatus::OK);
    return true;
}

bool IMUSensor::initializeBNO08X() {
    std::cout << name_ << ": BNO08x initialization (to be implemented)" << std::endl;
    return false; 
}

bool IMUSensor::readData() {
    if (!connected_) {
        return false;
    }
    
    try {
        bool success = false;
        switch (type_) {
            case IMUType::MPU6050:
                success = readMPU6050Data();
                break;
            case IMUType::MPU9250:
                success = readMPU9250Data();
                break;
            case IMUType::BNO055:
                success = readBNO055Data();
                break;
            case IMUType::LSM9DS1:
                success = readLSM9DS1Data();
                break;
            case IMUType::BNO08X:
                success = readBNO08XData();
                break;
        }
        
        if (success) {
            latestData_.timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count();
        }
        
        return success;
        
    } catch (const std::exception& e) {
        std::cerr << name_ << ": Data read error: " << e.what() << std::endl;
        return false;
    }
}

bool IMUSensor::readMPU6050Data() {
    try {
        uint8_t data[14];
        for (int i = 0; i < 14; i++) {
            data[i] = readRegister(MPU6050Registers::ACCEL_XOUT_H + i);
        }
        
        int16_t accel_x = (data[0] << 8) | data[1];
        int16_t accel_y = (data[2] << 8) | data[3];
        int16_t accel_z = (data[4] << 8) | data[5];
        
        int16_t temp = (data[6] << 8) | data[7];
        
        int16_t gyro_x = (data[8] << 8) | data[9];
        int16_t gyro_y = (data[10] << 8) | data[11];
        int16_t gyro_z = (data[12] << 8) | data[13];
        
        float accel_scale = 16384.0f; 
        float gyro_scale = 131.0f;    
        float temp_scale = 340.0f;    
        
        std::lock_guard<std::mutex> lock(dataMutex_);
        latestData_.accel_x = accel_x / accel_scale * 9.81f; 
        latestData_.accel_y = accel_y / accel_scale * 9.81f;
        latestData_.accel_z = accel_z / accel_scale * 9.81f;
        latestData_.temperature = temp / temp_scale + 36.53f; 
        latestData_.gyro_x = gyro_x / gyro_scale; 
        latestData_.gyro_y = gyro_y / gyro_scale;
        latestData_.gyro_z = gyro_z / gyro_scale;
        
        return true;
        
    } catch (const std::exception& e) {
        std::cerr << name_ << ": MPU6050 data read error: " << e.what() << std::endl;
        return false;
    }
}

bool IMUSensor::readMPU9250Data() {
    try {
        if (!readMPU6050Data()) {
            return false;
        }
        
        if (!readAK8963Data()) {
            std::cerr << name_ << ": AK8963 data read failed" << std::endl;
        }
        
        return true;
        
    } catch (const std::exception& e) {
        std::cerr << name_ << ": MPU9250 data read error: " << e.what() << std::endl;
        return false;
    }
}

bool IMUSensor::readAK8963Data() {
    try {
        uint8_t st1 = readRegister(AK8963_ADDRESS, AK8963Registers::ST1);
        if (!(st1 & 0x01)) {
            return false; 
        }
        
        uint8_t data[7];
        for (int i = 0; i < 7; i++) {
            data[i] = readRegister(AK8963_ADDRESS, AK8963Registers::XOUT_L + i);
        }
        
        if (data[6] & 0x08) {
            std::cerr << name_ << ": Magnetometer overflow detected" << std::endl;
            return false;
        }
        
        int16_t mag_x = (data[1] << 8) | data[0];
        int16_t mag_y = (data[3] << 8) | data[2];
        int16_t mag_z = (data[5] << 8) | data[4];
        
        float mag_scale = 10.0f * 4912.0f / 32760.0f; 
        
        std::lock_guard<std::mutex> lock(dataMutex_);
        latestData_.mag_x = (float)mag_x * mag_scale * magCalibration_[0];
        latestData_.mag_y = (float)mag_y * mag_scale * magCalibration_[1];
        latestData_.mag_z = (float)mag_z * mag_scale * magCalibration_[2];
        
        return true;
        
    } catch (const std::exception& e) {
        std::cerr << name_ << ": AK8963 data read error: " << e.what() << std::endl;
        return false;
    }
}

bool IMUSensor::readBNO055Data() {
    return false;
}

bool IMUSensor::readLSM9DS1Data() {
    return false;
}

bool IMUSensor::readBNO08XData() {
    std::lock_guard<std::mutex> lock(dataMutex_);
    latestData_.timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    return true;
}

bool IMUSensor::readBNO08XData() {
    return false;
}

IMUData IMUSensor::getLatestData() const {
    std::lock_guard<std::mutex> lock(dataMutex_);
    return latestData_;
}

IMUStatus IMUSensor::getStatus() const {
    return status_;
}

bool IMUSensor::isConnected() const {
    return connected_;
}

bool IMUSensor::isCalibrated() const {
    return calibrated_;
}

bool IMUSensor::writeRegister(uint8_t reg, uint8_t value) {
    return writeRegister(i2cAddr_, reg, value);
}

bool IMUSensor::writeRegister(uint8_t addr, uint8_t reg, uint8_t value) {
    if (!i2c_) {
        return false;
    }
    
    try {
        i2c_->writeRegister(addr, reg, value);
        return true;
    } catch (const std::exception& e) {
        std::cerr << name_ << ": Register write failed (addr=0x" << std::hex << (int)addr 
                  << ", reg=0x" << (int)reg << ", value=0x" << (int)value 
                  << "): " << e.what() << std::dec << std::endl;
        return false;
    }
}

uint8_t IMUSensor::readRegister(uint8_t reg) {
    return readRegister(i2cAddr_, reg);
}

uint8_t IMUSensor::readRegister(uint8_t addr, uint8_t reg) {
    if (!i2c_) {
        throw std::runtime_error("I2C interface is not initialized.");
    }
    
    try {
        return i2c_->readRegister(addr, reg);
    } catch (const std::exception& e) {
        std::cerr << name_ << ": Register read failed (addr=0x" << std::hex << (int)addr 
                  << ", reg=0x" << (int)reg << "): " << e.what() << std::dec << std::endl;
        throw;
    }
}

bool IMUSensor::testConnection() {
    try {
        if (type_ == IMUType::BNO08X) {
            return true;
        }
        uint8_t who_am_i = readRegister(MPU6050Registers::WHO_AM_I);
        return who_am_i == 0x68; 
    } catch (const std::exception& e) {
        return false;
    }
}

void IMUSensor::updateStatus(IMUStatus newStatus) {
    status_ = newStatus;
    
    if (newStatus != IMUStatus::OK) {
        std::cerr << name_ << ": Status change - ";
        switch (newStatus) {
            case IMUStatus::ERROR_I2C:
                std::cerr << "I2C interface error";
                break;
            case IMUStatus::ERROR_CALIBRATION:
                std::cerr << "Calibration error";
                break;
            case IMUStatus::ERROR_COMMUNICATION:
                std::cerr << "Communication error";
                break;
            default:
                std::cerr << "Unknown error";
                break;
        }
        std::cerr << std::endl;
    }
}

bool IMUSensor::calibrate() { return true; }
bool IMUSensor::reset() { return initialize(); }
bool IMUSensor::setSampleRate(uint16_t rate) { sampleRate_ = rate; return true; }
bool IMUSensor::setGyroRange(uint16_t range) { gyroRange_ = range; return true; }
bool IMUSensor::setAccelRange(uint8_t range) { accelRange_ = range; return true; }
void IMUSensor::enableLowPassFilter(bool enable) { lowPassFilterEnabled_ = enable; }
void IMUSensor::setFilterCutoff(float cutoff) { filterCutoff_ = cutoff; }
bool IMUSensor::startCalibration() { calibrating_ = true; return true; }
bool IMUSensor::stopCalibration() { calibrating_ = false; return true; }
float IMUSensor::getCalibrationProgress() const { return calibrationProgress_; }

} 