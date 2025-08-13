#pragma once
#include <cstdint>
#include <cstddef>

class I2CInterface {
public:
    virtual ~I2CInterface() = default;

    virtual void writeRegister(uint8_t devAddr, uint8_t regAddr, uint8_t value) = 0;
    virtual uint8_t readRegister(uint8_t devAddr, uint8_t regAddr) = 0;

    // Raw I2C read/write (no register address prefix). Needed for devices like BNO08x
    virtual void writeBytes(uint8_t devAddr, const uint8_t* data, size_t length) = 0;
    virtual void readBytes(uint8_t devAddr, uint8_t* buffer, size_t length) = 0;
};
