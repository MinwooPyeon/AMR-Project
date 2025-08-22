#include "amr/i2c/i2c_address_manager.h"
#include <iostream>
#include <algorithm>
#include <iomanip>

namespace amr {

I2CAddressManager::I2CAddressManager(std::shared_ptr<I2CInterface> i2c)
    : i2c_(std::move(i2c))
{
    if (!i2c_) {
        throw std::invalid_argument("I2C 인터페이스가 null입니다.");
    }
    
    std::cout << "I2C address manager initialization completed" << std::endl;
}

uint8_t I2CAddressManager::findAvailableAddress(const std::string& deviceName) {
    std::cout << deviceName << "의 사용 가능한 I2C 주소 찾는 중..." << std::endl;
    
    for (uint8_t addr : defaultAddresses_) {
        if (isAddressAvailable(addr)) {
            if (reserveAddress(addr, deviceName)) {
                std::cout << deviceName << " address assignment completed: 0x" 
                          << std::hex << (int)addr << std::dec << std::endl;
                return addr;
            }
        }
    }
    
    std::cerr << deviceName << "의 사용 가능한 I2C 주소를 찾을 수 없습니다." << std::endl;
    return 0;
}

std::vector<uint8_t> I2CAddressManager::findAvailableAddresses(int count) {
    std::vector<uint8_t> addresses;
    std::cout << count << "개의 사용 가능한 I2C 주소 찾는 중..." << std::endl;
    
    for (uint8_t addr : defaultAddresses_) {
        if (addresses.size() >= static_cast<size_t>(count)) {
            break;
        }
        
        if (isAddressAvailable(addr)) {
            std::string deviceName = "Device_" + std::to_string(addresses.size());
            if (reserveAddress(addr, deviceName)) {
                addresses.push_back(addr);
                std::cout << deviceName << " 주소 할당: 0x" 
                          << std::hex << (int)addr << std::dec << std::endl;
            }
        }
    }
    
    if (addresses.size() < static_cast<size_t>(count)) {
        std::cerr << "요청된 " << count << "개의 주소 중 " 
                  << addresses.size() << "개만 할당되었습니다." << std::endl;
    }
    
    return addresses;
}

bool I2CAddressManager::isAddressAvailable(uint8_t address) {

    if (reservedAddresses_.find(address) != reservedAddresses_.end()) {
        return false;
    }
    

    return testAddress(address);
}

bool I2CAddressManager::testAddress(uint8_t address) {
    try {

        uint8_t testValue = i2c_->readRegister(address, 0x00);
        return true;
    } catch (const std::exception& e) {
        return false;
    }
}

bool I2CAddressManager::reserveAddress(uint8_t address, const std::string& deviceName) {
    if (reservedAddresses_.find(address) != reservedAddresses_.end()) {
        std::cerr << "Address 0x" << std::hex << (int)address 
                  << std::dec << " is already in use." << std::endl;
        return false;
    }
    
    reservedAddresses_.insert(address);
    deviceNames_[address] = deviceName;
    
    std::cout << "Address 0x" << std::hex << (int)address 
              << std::dec << " reserved for " << deviceName << "." << std::endl;
    return true;
}

bool I2CAddressManager::releaseAddress(uint8_t address) {
    auto it = reservedAddresses_.find(address);
    if (it == reservedAddresses_.end()) {
        std::cerr << "Address 0x" << std::hex << (int)address 
                  << std::dec << " is not reserved." << std::endl;
        return false;
    }
    
    std::string deviceName = deviceNames_[address];
    reservedAddresses_.erase(it);
    deviceNames_.erase(address);
    
    std::cout << "Address 0x" << std::hex << (int)address 
              << std::dec << " released from " << deviceName << "." << std::endl;
    return true;
}

std::set<uint8_t> I2CAddressManager::getReservedAddresses() const {
    return reservedAddresses_;
}

std::string I2CAddressManager::getDeviceName(uint8_t address) const {
    auto it = deviceNames_.find(address);
    if (it != deviceNames_.end()) {
        return it->second;
    }
    return "Unknown";
}

std::vector<uint8_t> I2CAddressManager::scanBus() {
    std::vector<uint8_t> foundAddresses;
    std::cout << "I2C bus scan started..." << std::endl;
    
    for (uint8_t addr : defaultAddresses_) {
        if (testAddress(addr)) {
            foundAddresses.push_back(addr);
            std::cout << "Found address: 0x" << std::hex << (int)addr << std::dec << std::endl;
        }
    }
    
    std::cout << "Total " << foundAddresses.size() << " I2C devices found." << std::endl;
    return foundAddresses;
}

void I2CAddressManager::printBusStatus() {
    std::cout << "\n=== I2C Bus Status ===" << std::endl;
    

    std::vector<uint8_t> allDevices = scanBus();
    
    std::cout << "\nReserved addresses:" << std::endl;
    for (uint8_t addr : reservedAddresses_) {
        std::string deviceName = getDeviceName(addr);
        std::cout << "  0x" << std::hex << (int)addr << std::dec 
                  << " -> " << deviceName << std::endl;
    }
    
    std::cout << "\nAvailable addresses:" << std::endl;
    for (uint8_t addr : allDevices) {
        if (reservedAddresses_.find(addr) == reservedAddresses_.end()) {
            std::cout << "  0x" << std::hex << (int)addr << std::dec << std::endl;
        }
    }
    
    std::cout << "\n사용 불가능한 주소:" << std::endl;
    for (uint8_t addr : defaultAddresses_) {
        if (std::find(allDevices.begin(), allDevices.end(), addr) == allDevices.end()) {
            std::cout << "  0x" << std::hex << (int)addr << std::dec << std::endl;
        }
    }
    
    std::cout << "===================" << std::endl;
}

} // namespace amr 