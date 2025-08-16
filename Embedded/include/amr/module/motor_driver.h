#pragma once
#include <memory>
#include <cstdint>
#include <string>
#include <atomic>
#include <mutex>
#include "amr/i2c/i2c_interface.h"

namespace amr {

namespace MotorConstants {
    const uint8_t DEFAULT_I2C_ADDRESS = 0x60;  
    const uint8_t REG_MOTOR_A_SPEED = 0x00;    
    const uint8_t REG_MOTOR_A_DIRECTION = 0x01; 
    const uint8_t REG_MOTOR_B_SPEED = 0x02;    
    const uint8_t REG_MOTOR_B_DIRECTION = 0x03; 
    const uint8_t REG_CONFIG = 0x04;            
    const uint8_t REG_STATUS = 0x05;            
}

enum class MotorDirection {
    STOP = 0,
    FORWARD = 1,
    REVERSE = 2
};

enum class MotorStatus {
    OK = 0,
    ERROR_I2C = 1,
    ERROR_ADDRESS = 2,
    ERROR_COMMUNICATION = 3
};

class MotorDriver {
public:
    MotorDriver(std::shared_ptr<I2CInterface> i2c, uint8_t i2cAddr = MotorConstants::DEFAULT_I2C_ADDRESS, const std::string& name = "Motor");
    ~MotorDriver();

    bool setSpeed(int speed);  
    bool stop();
    bool emergencyStop();
    
    bool setMotorASpeed(int speed);  
    bool setMotorBSpeed(int speed);  
    bool setSpeed(int motorA, int motorB);  
    
    MotorStatus getStatus() const;
    int getCurrentSpeed() const;
    int getMotorASpeed() const;
    int getMotorBSpeed() const;
    MotorDirection getCurrentDirection() const;
    bool isConnected() const;
    
    bool setI2CAddress(uint8_t newAddr);
    uint8_t getI2CAddress() const;
    std::string getName() const;
    
    bool testConnection();
    bool reset();
    bool readStatus();

private:
    std::shared_ptr<I2CInterface> i2c_;
    uint8_t i2cAddr_;
    std::string name_;
    
    mutable std::mutex mutex_;
    std::atomic<int> currentSpeed_{0};
    std::atomic<int> motorASpeed_{0};
    std::atomic<int> motorBSpeed_{0};
    std::atomic<MotorDirection> currentDirection_{MotorDirection::STOP};
    std::atomic<MotorStatus> status_{MotorStatus::OK};
    std::atomic<bool> connected_{false};
    
    bool writeRegister(uint8_t reg, uint8_t value);
    uint8_t readRegister(uint8_t reg);
    bool checkAddressAvailability();
    void updateStatus(MotorStatus newStatus);
    MotorDirection speedToDirection(int speed);
    uint8_t speedToPWM(int speed);
};

} 