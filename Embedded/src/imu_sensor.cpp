#include "amr/imu_sensor.h"
#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>
#include <algorithm>

namespace amr {

// MPU6050/MPU9250 레지스터 정의
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

// AK8963 자력계 레지스터 정의 (MPU9250)
namespace AK8963Registers {
    const uint8_t WHO_AM_I = 0x00;      // should return 0x48
    const uint8_t INFO = 0x01;
    const uint8_t ST1 = 0x02;           // data ready status bit 0
    const uint8_t XOUT_L = 0x03;        // data
    const uint8_t XOUT_H = 0x04;
    const uint8_t YOUT_L = 0x05;
    const uint8_t YOUT_H = 0x06;
    const uint8_t ZOUT_L = 0x07;
    const uint8_t ZOUT_H = 0x08;
    const uint8_t ST2 = 0x09;           // Data overflow bit 3 and data read error status bit 2
    const uint8_t CNTL = 0x0A;          // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
    const uint8_t ASTC = 0x0C;          // Self test control
    const uint8_t I2CDIS = 0x0F;        // I2C disable
    const uint8_t ASAX = 0x10;          // Fuse ROM x-axis sensitivity adjustment value
    const uint8_t ASAY = 0x11;          // Fuse ROM y-axis sensitivity adjustment value
    const uint8_t ASAZ = 0x12;          // Fuse ROM z-axis sensitivity adjustment value
}

// MPU9250 자력계 주소
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
    
    // 데이터 초기화
    latestData_ = {};
    latestData_.timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    
    std::cout << name_ << ": IMU 센서 초기화 시작 (타입: " << static_cast<int>(type_) 
              << ", 주소: 0x" << std::hex << (int)i2cAddr_ << std::dec << ")" << std::endl;
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
        // 연결 테스트
        if (!testConnection()) {
            updateStatus(IMUStatus::ERROR_COMMUNICATION);
            return false;
        }
        
        // 센서 타입별 초기화
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
        }
        
        if (success) {
            connected_ = true;
            updateStatus(IMUStatus::OK);
            std::cout << name_ << ": IMU 센서 초기화 완료" << std::endl;
            return true;
        } else {
            updateStatus(IMUStatus::ERROR_COMMUNICATION);
            return false;
        }
        
    } catch (const std::exception& e) {
        std::cerr << name_ << ": 초기화 오류: " << e.what() << std::endl;
        updateStatus(IMUStatus::ERROR_COMMUNICATION);
        return false;
    }
}

bool IMUSensor::initializeMPU6050() {
    try {
        // WHO_AM_I 확인
        uint8_t who_am_i = readRegister(MPU6050Registers::WHO_AM_I);
        if (who_am_i != 0x68) {
            std::cerr << name_ << ": WHO_AM_I 불일치 (예상: 0x68, 실제: 0x" 
                      << std::hex << (int)who_am_i << std::dec << ")" << std::endl;
            return false;
        }
        
        // 파워 매니지먼트 리셋
        writeRegister(MPU6050Registers::PWR_MGMT_1, 0x80);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        // 클록 소스 설정 (PLL with X axis gyroscope reference)
        writeRegister(MPU6050Registers::PWR_MGMT_1, 0x01);
        
        // 샘플레이트 설정
        writeRegister(MPU6050Registers::SMPLRT_DIV, 0x07); // 125Hz
        
        // 설정 레지스터 (DLPF 설정)
        writeRegister(MPU6050Registers::CONFIG, 0x06); // 5Hz DLPF
        
        // 자이로스코프 설정
        uint8_t gyro_config = 0;
        switch (gyroRange_) {
            case 250: gyro_config = 0x00; break;
            case 500: gyro_config = 0x08; break;
            case 1000: gyro_config = 0x10; break;
            case 2000: gyro_config = 0x18; break;
            default: gyro_config = 0x00; break;
        }
        writeRegister(MPU6050Registers::GYRO_CONFIG, gyro_config);
        
        // 가속도계 설정
        uint8_t accel_config = 0;
        switch (accelRange_) {
            case 2: accel_config = 0x00; break;
            case 4: accel_config = 0x08; break;
            case 8: accel_config = 0x10; break;
            case 16: accel_config = 0x18; break;
            default: accel_config = 0x00; break;
        }
        writeRegister(MPU6050Registers::ACCEL_CONFIG, accel_config);
        
        // 인터럽트 활성화
        writeRegister(MPU6050Registers::INT_ENABLE, 0x01);
        
        std::cout << name_ << ": MPU6050 초기화 완료" << std::endl;
        return true;
        
    } catch (const std::exception& e) {
        std::cerr << name_ << ": MPU6050 초기화 오류: " << e.what() << std::endl;
        return false;
    }
}

bool IMUSensor::initializeMPU9250() {
    try {
        // MPU9250 기본 초기화 (MPU6050과 동일)
        if (!initializeMPU6050()) {
            return false;
        }
        
        // MPU9250 특별 설정
        // 인터럽트 핀 설정 및 I2C 바이패스 활성화
        writeRegister(MPU6050Registers::INT_PIN_CFG, 0x22);
        writeRegister(MPU6050Registers::INT_ENABLE, 0x01);
        
        // 자력계(AK8963) 초기화
        if (!initializeAK8963()) {
            std::cerr << name_ << ": AK8963 자력계 초기화 실패" << std::endl;
            return false;
        }
        
        std::cout << name_ << ": MPU9250 초기화 완료 (자력계 포함)" << std::endl;
        return true;
        
    } catch (const std::exception& e) {
        std::cerr << name_ << ": MPU9250 초기화 오류: " << e.what() << std::endl;
        return false;
    }
}

bool IMUSensor::initializeAK8963() {
    try {
        // AK8963 WHO_AM_I 확인
        uint8_t who_am_i = readRegister(AK8963_ADDRESS, AK8963Registers::WHO_AM_I);
        if (who_am_i != 0x48) {
            std::cerr << name_ << ": AK8963 WHO_AM_I 불일치 (예상: 0x48, 실제: 0x" 
                      << std::hex << (int)who_am_i << std::dec << ")" << std::endl;
            return false;
        }
        
        // 자력계 파워 다운
        writeRegister(AK8963_ADDRESS, AK8963Registers::CNTL, 0x00);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        
        // Fuse ROM 액세스 모드
        writeRegister(AK8963_ADDRESS, AK8963Registers::CNTL, 0x0F);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        
        // 캘리브레이션 값 읽기
        uint8_t asa[3];
        for (int i = 0; i < 3; i++) {
            asa[i] = readRegister(AK8963_ADDRESS, AK8963Registers::ASAX + i);
        }
        
        // 캘리브레이션 값 저장 (나중에 사용)
        magCalibration_[0] = (float)(asa[0] - 128) / 256.0f + 1.0f;
        magCalibration_[1] = (float)(asa[1] - 128) / 256.0f + 1.0f;
        magCalibration_[2] = (float)(asa[2] - 128) / 256.0f + 1.0f;
        
        // 파워 다운
        writeRegister(AK8963_ADDRESS, AK8963Registers::CNTL, 0x00);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        
        // 16비트 해상도, 8Hz 샘플레이트로 설정
        writeRegister(AK8963_ADDRESS, AK8963Registers::CNTL, 0x16);
        
        std::cout << name_ << ": AK8963 자력계 초기화 완료" << std::endl;
        return true;
        
    } catch (const std::exception& e) {
        std::cerr << name_ << ": AK8963 초기화 오류: " << e.what() << std::endl;
        return false;
    }
}

bool IMUSensor::initializeBNO055() {
    // BNO055는 Bosch의 9축 센서
    std::cout << name_ << ": BNO055 초기화 (구현 예정)" << std::endl;
    return false; // 아직 구현되지 않음
}

bool IMUSensor::initializeLSM9DS1() {
    // LSM9DS1는 STMicroelectronics의 9축 센서
    std::cout << name_ << ": LSM9DS1 초기화 (구현 예정)" << std::endl;
    return false; // 아직 구현되지 않음
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
        }
        
        if (success) {
            latestData_.timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count();
        }
        
        return success;
        
    } catch (const std::exception& e) {
        std::cerr << name_ << ": 데이터 읽기 오류: " << e.what() << std::endl;
        return false;
    }
}

bool IMUSensor::readMPU6050Data() {
    try {
        // 14바이트 데이터 읽기 (가속도 6바이트 + 온도 2바이트 + 자이로 6바이트)
        uint8_t data[14];
        for (int i = 0; i < 14; i++) {
            data[i] = readRegister(MPU6050Registers::ACCEL_XOUT_H + i);
        }
        
        // 가속도 데이터 변환 (16비트, 2의 보수)
        int16_t accel_x = (data[0] << 8) | data[1];
        int16_t accel_y = (data[2] << 8) | data[3];
        int16_t accel_z = (data[4] << 8) | data[5];
        
        // 온도 데이터 변환
        int16_t temp = (data[6] << 8) | data[7];
        
        // 자이로스코프 데이터 변환
        int16_t gyro_x = (data[8] << 8) | data[9];
        int16_t gyro_y = (data[10] << 8) | data[11];
        int16_t gyro_z = (data[12] << 8) | data[13];
        
        // 스케일 팩터 적용
        float accel_scale = 16384.0f; // ±2g 기준
        float gyro_scale = 131.0f;    // ±250°/s 기준
        float temp_scale = 340.0f;    // 온도 스케일
        
        std::lock_guard<std::mutex> lock(dataMutex_);
        latestData_.accel_x = accel_x / accel_scale * 9.81f; // m/s²
        latestData_.accel_y = accel_y / accel_scale * 9.81f;
        latestData_.accel_z = accel_z / accel_scale * 9.81f;
        latestData_.temperature = temp / temp_scale + 36.53f; // 섭씨
        latestData_.gyro_x = gyro_x / gyro_scale; // 도/초
        latestData_.gyro_y = gyro_y / gyro_scale;
        latestData_.gyro_z = gyro_z / gyro_scale;
        
        return true;
        
    } catch (const std::exception& e) {
        std::cerr << name_ << ": MPU6050 데이터 읽기 오류: " << e.what() << std::endl;
        return false;
    }
}

bool IMUSensor::readMPU9250Data() {
    try {
        // MPU6050과 동일한 방식으로 가속도, 자이로, 온도 읽기
        if (!readMPU6050Data()) {
            return false;
        }
        
        // 자력계 데이터 읽기
        if (!readAK8963Data()) {
            std::cerr << name_ << ": AK8963 데이터 읽기 실패" << std::endl;
            // 자력계 실패해도 다른 데이터는 사용 가능
        }
        
        return true;
        
    } catch (const std::exception& e) {
        std::cerr << name_ << ": MPU9250 데이터 읽기 오류: " << e.what() << std::endl;
        return false;
    }
}

bool IMUSensor::readAK8963Data() {
    try {
        // 데이터 준비 상태 확인
        uint8_t st1 = readRegister(AK8963_ADDRESS, AK8963Registers::ST1);
        if (!(st1 & 0x01)) {
            return false; // 데이터가 준비되지 않음
        }
        
        // 7바이트 읽기 (X, Y, Z + ST2)
        uint8_t data[7];
        for (int i = 0; i < 7; i++) {
            data[i] = readRegister(AK8963_ADDRESS, AK8963Registers::XOUT_L + i);
        }
        
        // ST2 확인 (오버플로우 체크)
        if (data[6] & 0x08) {
            std::cerr << name_ << ": 자력계 오버플로우 감지" << std::endl;
            return false;
        }
        
        // 16비트 데이터 변환 (Little Endian)
        int16_t mag_x = (data[1] << 8) | data[0];
        int16_t mag_y = (data[3] << 8) | data[2];
        int16_t mag_z = (data[5] << 8) | data[4];
        
        // 스케일 팩터 적용 (16비트 해상도, ±4900 μT)
        float mag_scale = 10.0f * 4912.0f / 32760.0f; // μT per LSB
        
        std::lock_guard<std::mutex> lock(dataMutex_);
        latestData_.mag_x = (float)mag_x * mag_scale * magCalibration_[0];
        latestData_.mag_y = (float)mag_y * mag_scale * magCalibration_[1];
        latestData_.mag_z = (float)mag_z * mag_scale * magCalibration_[2];
        
        return true;
        
    } catch (const std::exception& e) {
        std::cerr << name_ << ": AK8963 데이터 읽기 오류: " << e.what() << std::endl;
        return false;
    }
}

bool IMUSensor::readBNO055Data() {
    // BNO055 구현 예정
    return false;
}

bool IMUSensor::readLSM9DS1Data() {
    // LSM9DS1 구현 예정
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
        std::cerr << name_ << ": 레지스터 쓰기 실패 (addr=0x" << std::hex << (int)addr 
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
        throw std::runtime_error("I2C 인터페이스가 초기화되지 않았습니다.");
    }
    
    try {
        return i2c_->readRegister(addr, reg);
    } catch (const std::exception& e) {
        std::cerr << name_ << ": 레지스터 읽기 실패 (addr=0x" << std::hex << (int)addr 
                  << ", reg=0x" << (int)reg << "): " << e.what() << std::dec << std::endl;
        throw;
    }
}

bool IMUSensor::testConnection() {
    try {
        uint8_t who_am_i = readRegister(MPU6050Registers::WHO_AM_I);
        return who_am_i == 0x68; // MPU6050/MPU9250의 WHO_AM_I 값
    } catch (const std::exception& e) {
        return false;
    }
}

void IMUSensor::updateStatus(IMUStatus newStatus) {
    status_ = newStatus;
    
    if (newStatus != IMUStatus::OK) {
        std::cerr << name_ << ": 상태 변경 - ";
        switch (newStatus) {
            case IMUStatus::ERROR_I2C:
                std::cerr << "I2C 인터페이스 오류";
                break;
            case IMUStatus::ERROR_CALIBRATION:
                std::cerr << "캘리브레이션 오류";
                break;
            case IMUStatus::ERROR_COMMUNICATION:
                std::cerr << "통신 오류";
                break;
            default:
                std::cerr << "알 수 없는 오류";
                break;
        }
        std::cerr << std::endl;
    }
}

// 나머지 메서드들은 기본 구현
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

} // namespace amr 