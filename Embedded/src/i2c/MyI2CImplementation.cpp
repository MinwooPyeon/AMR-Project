#include "amr/i2c/MyI2CImplementation.h"
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <stdexcept>
#include <cstdio> // perror

MyI2CImplementation::MyI2CImplementation(const std::string& device)
    : device_(device), fd_(-1)
{
    fd_ = open(device_.c_str(), O_RDWR);
    if (fd_ < 0) {
        perror(("Failed to open I2C device " + device_).c_str());
        throw std::runtime_error("Failed to open I2C device");
    }
}

MyI2CImplementation::~MyI2CImplementation()
{
    if (fd_ >= 0) {
        close(fd_);
    }
}

void MyI2CImplementation::setSlaveAddress(uint8_t addr)
{
    int ret = ioctl(fd_, I2C_SLAVE, addr);
    if (ret < 0) {
        perror("ioctl I2C_SLAVE failed");
        throw std::runtime_error("Failed to set I2C slave address");
    }
}

void MyI2CImplementation::writeRegister(uint8_t devAddr, uint8_t regAddr, uint8_t value)
{
    setSlaveAddress(devAddr);
    uint8_t buf[2] = { regAddr, value };
    if (write(fd_, buf, 2) != 2) {
        perror("Failed to write to I2C device");
        throw std::runtime_error("Failed to write to I2C device");
    }
}

uint8_t MyI2CImplementation::readRegister(uint8_t devAddr, uint8_t regAddr)
{
    setSlaveAddress(devAddr);
    if (write(fd_, &regAddr, 1) != 1) {
        perror("Failed to write register address to I2C device");
        throw std::runtime_error("Failed to write register address");
    }
    uint8_t value = 0;
    if (read(fd_, &value, 1) != 1) {
        perror("Failed to read from I2C device");
        throw std::runtime_error("Failed to read from I2C device");
    }
    return value;
}

void MyI2CImplementation::writeBytes(uint8_t devAddr, const uint8_t* data, size_t length)
{
    setSlaveAddress(devAddr);
    if (length == 0) return;
    ssize_t written = write(fd_, data, static_cast<unsigned int>(length));
    if (written < 0 || static_cast<size_t>(written) != length) {
        perror("Failed to write raw bytes to I2C device");
        throw std::runtime_error("Failed to write raw bytes to I2C device");
    }
}

void MyI2CImplementation::readBytes(uint8_t devAddr, uint8_t* buffer, size_t length)
{
    setSlaveAddress(devAddr);
    if (length == 0) return;
    ssize_t readLen = read(fd_, buffer, static_cast<unsigned int>(length));
    if (readLen < 0 || static_cast<size_t>(readLen) != length) {
        perror("Failed to read raw bytes from I2C device");
        throw std::runtime_error("Failed to read raw bytes from I2C device");
    }
}