#pragma once
#include "i2c_interface.h"
#include <string>

class MyI2CImplementation : public I2CInterface {
public:
    explicit MyI2CImplementation(const std::string& device = "/dev/i2c-1");
    ~MyI2CImplementation() override;

    void writeRegister(uint8_t devAddr, uint8_t regAddr, uint8_t value) override;
    uint8_t readRegister(uint8_t devAddr, uint8_t regAddr) override;
    void writeBytes(uint8_t devAddr, const uint8_t* data, size_t length) override;
    void readBytes(uint8_t devAddr, uint8_t* buffer, size_t length) override;

private:
    int fd_;
    std::string device_;

    void setSlaveAddress(uint8_t addr);
};
