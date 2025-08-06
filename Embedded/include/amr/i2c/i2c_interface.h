#pragma once
#include <cstdint>

class I2CInterface {
public:
    virtual ~I2CInterface() = default;

    virtual void writeRegister(uint8_t devAddr, uint8_t regAddr, uint8_t value) = 0;
    virtual uint8_t readRegister(uint8_t devAddr, uint8_t regAddr) = 0;
};
