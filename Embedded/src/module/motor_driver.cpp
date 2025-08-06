#include "amr/module/motor_driver.h"
#include <iostream>
#include <cmath>
#include <chrono>
#include <thread>

namespace amr {

MotorDriver::MotorDriver(std::shared_ptr<I2CInterface> i2c, uint8_t i2cAddr, const std::string& name)
    : i2c_(std::move(i2c)), i2cAddr_(i2cAddr), name_(name)
{
    if (!i2c_) {
        updateStatus(MotorStatus::ERROR_I2C);
        return;
    }
    
    // 초기 연결 테스트
    if (testConnection()) {
        connected_ = true;
        updateStatus(MotorStatus::OK);
        stop(); // 안전을 위해 정지 상태로 초기화
        std::cout << name_ << ": Motor Driver 초기화 완료 (주소: 0x" 
                  << std::hex << (int)i2cAddr_ << std::dec << ")" << std::endl;
    } else {
        updateStatus(MotorStatus::ERROR_COMMUNICATION);
        std::cerr << name_ << ": Motor Driver 연결 실패" << std::endl;
    }
}

MotorDriver::~MotorDriver() {
    emergencyStop();
}

bool MotorDriver::setSpeed(int speed) {
    // 기본 setSpeed는 모터 A에 적용 (하위 호환성)
    return setMotorASpeed(speed);
}

bool MotorDriver::setMotorASpeed(int speed) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!connected_ || status_ != MotorStatus::OK) {
        std::cerr << name_ << ": 모터 A가 연결되지 않았거나 오류 상태입니다." << std::endl;
        return false;
    }
    
    // 속도 범위 제한
    if (speed > 100) speed = 100;
    else if (speed < -100) speed = -100;
    
    try {
        // 방향 설정
        MotorDirection direction = speedToDirection(speed);
        if (!writeRegister(MotorConstants::REG_MOTOR_A_DIRECTION, static_cast<uint8_t>(direction))) {
            updateStatus(MotorStatus::ERROR_COMMUNICATION);
            return false;
        }
        
        // 속도 설정 (PWM 값으로 변환)
        uint8_t pwmValue = speedToPWM(speed);
        if (!writeRegister(MotorConstants::REG_MOTOR_A_SPEED, pwmValue)) {
            updateStatus(MotorStatus::ERROR_COMMUNICATION);
            return false;
        }
        
        // 상태 업데이트
        motorASpeed_ = speed;
        currentSpeed_ = speed; // 하위 호환성
        currentDirection_ = direction;
        updateStatus(MotorStatus::OK);
        
        return true;
    } catch (const std::exception& e) {
        std::cerr << name_ << ": 모터 A setSpeed 오류: " << e.what() << std::endl;
        updateStatus(MotorStatus::ERROR_COMMUNICATION);
        return false;
    }
}

bool MotorDriver::setMotorBSpeed(int speed) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!connected_ || status_ != MotorStatus::OK) {
        std::cerr << name_ << ": 모터 B가 연결되지 않았거나 오류 상태입니다." << std::endl;
        return false;
    }
    
    // 속도 범위 제한
    if (speed > 100) speed = 100;
    else if (speed < -100) speed = -100;
    
    try {
        // 방향 설정
        MotorDirection direction = speedToDirection(speed);
        if (!writeRegister(MotorConstants::REG_MOTOR_B_DIRECTION, static_cast<uint8_t>(direction))) {
            updateStatus(MotorStatus::ERROR_COMMUNICATION);
            return false;
        }
        
        // 속도 설정 (PWM 값으로 변환)
        uint8_t pwmValue = speedToPWM(speed);
        if (!writeRegister(MotorConstants::REG_MOTOR_B_SPEED, pwmValue)) {
            updateStatus(MotorStatus::ERROR_COMMUNICATION);
            return false;
        }
        
        // 상태 업데이트
        motorBSpeed_ = speed;
        updateStatus(MotorStatus::OK);
        
        return true;
    } catch (const std::exception& e) {
        std::cerr << name_ << ": 모터 B setSpeed 오류: " << e.what() << std::endl;
        updateStatus(MotorStatus::ERROR_COMMUNICATION);
        return false;
    }
}

bool MotorDriver::setSpeed(int motorA, int motorB) {
    bool successA = setMotorASpeed(motorA);
    bool successB = setMotorBSpeed(motorB);
    return successA && successB;
}

bool MotorDriver::stop() {
    return setSpeed(0);
}

bool MotorDriver::emergencyStop() {
    std::lock_guard<std::mutex> lock(mutex_);
    
    try {
        // 즉시 정지 명령 (양쪽 모터 모두)
        if (!writeRegister(MotorConstants::REG_MOTOR_A_DIRECTION, static_cast<uint8_t>(MotorDirection::STOP))) {
            updateStatus(MotorStatus::ERROR_COMMUNICATION);
            return false;
        }
        
        if (!writeRegister(MotorConstants::REG_MOTOR_B_DIRECTION, static_cast<uint8_t>(MotorDirection::STOP))) {
            updateStatus(MotorStatus::ERROR_COMMUNICATION);
            return false;
        }
        
        if (!writeRegister(MotorConstants::REG_MOTOR_A_SPEED, 0)) {
            updateStatus(MotorStatus::ERROR_COMMUNICATION);
            return false;
        }
        
        if (!writeRegister(MotorConstants::REG_MOTOR_B_SPEED, 0)) {
            updateStatus(MotorStatus::ERROR_COMMUNICATION);
            return false;
        }
        
        currentSpeed_ = 0;
        motorASpeed_ = 0;
        motorBSpeed_ = 0;
        currentDirection_ = MotorDirection::STOP;
        updateStatus(MotorStatus::OK);
        
        return true;
    } catch (const std::exception& e) {
        std::cerr << name_ << ": emergencyStop 오류: " << e.what() << std::endl;
        updateStatus(MotorStatus::ERROR_COMMUNICATION);
        return false;
    }
}

MotorStatus MotorDriver::getStatus() const {
    return status_;
}

int MotorDriver::getCurrentSpeed() const {
    return currentSpeed_;
}

int MotorDriver::getMotorASpeed() const {
    return motorASpeed_;
}

int MotorDriver::getMotorBSpeed() const {
    return motorBSpeed_;
}

MotorDirection MotorDriver::getCurrentDirection() const {
    return currentDirection_;
}

bool MotorDriver::isConnected() const {
    return connected_;
}

bool MotorDriver::setI2CAddress(uint8_t newAddr) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (newAddr == i2cAddr_) {
        return true; // 같은 주소면 변경 불필요
    }
    
    // 새 주소가 사용 가능한지 확인
    uint8_t oldAddr = i2cAddr_;
    i2cAddr_ = newAddr;
    
    if (checkAddressAvailability()) {
        // 주소 변경 명령 전송 (일부 모터 드라이버에서 지원)
        try {
            writeRegister(0xFF, newAddr); // 주소 변경 레지스터 (실제 하드웨어에 따라 다를 수 있음)
            std::cout << name_ << ": I2C 주소를 0x" << std::hex << (int)oldAddr 
                      << "에서 0x" << (int)newAddr << "로 변경했습니다." << std::dec << std::endl;
            return true;
        } catch (const std::exception& e) {
            std::cerr << name_ << ": 주소 변경 실패: " << e.what() << std::endl;
            i2cAddr_ = oldAddr; // 원래 주소로 복원
            return false;
        }
    } else {
        i2cAddr_ = oldAddr; // 원래 주소로 복원
        return false;
    }
}

uint8_t MotorDriver::getI2CAddress() const {
    return i2cAddr_;
}

std::string MotorDriver::getName() const {
    return name_;
}

bool MotorDriver::testConnection() {
    try {
        // 상태 레지스터 읽기 테스트
        uint8_t testValue = readRegister(MotorConstants::REG_STATUS);
        connected_ = true;
        return true;
    } catch (const std::exception& e) {
        std::cerr << name_ << ": 연결 테스트 실패: " << e.what() << std::endl;
        connected_ = false;
        return false;
    }
}

bool MotorDriver::reset() {
    std::lock_guard<std::mutex> lock(mutex_);
    
    try {
        // 설정 레지스터를 통해 리셋
        writeRegister(MotorConstants::REG_CONFIG, 0x00);
        
        // 잠시 대기
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        // 상태 초기화
        currentSpeed_ = 0;
        motorASpeed_ = 0;
        motorBSpeed_ = 0;
        currentDirection_ = MotorDirection::STOP;
        
        // 연결 재확인
        if (testConnection()) {
            updateStatus(MotorStatus::OK);
            return true;
        } else {
            updateStatus(MotorStatus::ERROR_COMMUNICATION);
            return false;
        }
    } catch (const std::exception& e) {
        std::cerr << name_ << ": 리셋 실패: " << e.what() << std::endl;
        updateStatus(MotorStatus::ERROR_COMMUNICATION);
        return false;
    }
}

bool MotorDriver::readStatus() {
    try {
        uint8_t status = readRegister(MotorConstants::REG_STATUS);
        std::cout << name_ << ": 상태 레지스터 값: 0x" << std::hex << (int)status << std::dec << std::endl;
        return true;
    } catch (const std::exception& e) {
        std::cerr << name_ << ": 상태 읽기 실패: " << e.what() << std::endl;
        return false;
    }
}

bool MotorDriver::writeRegister(uint8_t reg, uint8_t value) {
    if (!i2c_) {
        return false;
    }
    
    try {
        i2c_->writeRegister(i2cAddr_, reg, value);
        return true;
    } catch (const std::exception& e) {
        std::cerr << name_ << ": 레지스터 쓰기 실패 (reg=0x" << std::hex << (int)reg 
                  << ", value=0x" << (int)value << "): " << e.what() << std::dec << std::endl;
        return false;
    }
}

uint8_t MotorDriver::readRegister(uint8_t reg) {
    if (!i2c_) {
        throw std::runtime_error("I2C 인터페이스가 초기화되지 않았습니다.");
    }
    
    try {
        return i2c_->readRegister(i2cAddr_, reg);
    } catch (const std::exception& e) {
        std::cerr << name_ << ": 레지스터 읽기 실패 (reg=0x" << std::hex << (int)reg 
                  << "): " << e.what() << std::dec << std::endl;
        throw;
    }
}

bool MotorDriver::checkAddressAvailability() {
    try {
        // 주소 사용 가능성 테스트
        uint8_t testValue = readRegister(MotorConstants::REG_STATUS);
        return true;
    } catch (const std::exception& e) {
        return false;
    }
}

void MotorDriver::updateStatus(MotorStatus newStatus) {
    status_ = newStatus;
    
    if (newStatus != MotorStatus::OK) {
        std::cerr << name_ << ": 상태 변경 - ";
        switch (newStatus) {
            case MotorStatus::ERROR_I2C:
                std::cerr << "I2C 인터페이스 오류";
                break;
            case MotorStatus::ERROR_ADDRESS:
                std::cerr << "I2C 주소 오류";
                break;
            case MotorStatus::ERROR_COMMUNICATION:
                std::cerr << "통신 오류";
                break;
            default:
                std::cerr << "알 수 없는 오류";
                break;
        }
        std::cerr << std::endl;
    }
}

MotorDirection MotorDriver::speedToDirection(int speed) {
    if (speed > 0) {
        return MotorDirection::FORWARD;
    } else if (speed < 0) {
        return MotorDirection::REVERSE;
    } else {
        return MotorDirection::STOP;
    }
}

uint8_t MotorDriver::speedToPWM(int speed) {
    // 속도를 PWM 값으로 변환 (0-255)
    int absSpeed = std::abs(speed);
    return static_cast<uint8_t>((absSpeed * 255) / 100);
}

} // namespace amr 