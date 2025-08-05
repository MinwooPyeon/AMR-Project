#include "amr/sysfsInterface/jetson_gpio.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include <cstring>
#include <map>

extern "C" {
#include <gpiod.h>
}

JetsonGpio::JetsonGpio() : chip_(nullptr), initialized_(false) {
    const char* chip_names[] = {"/dev/gpiochip0", "/dev/gpiochip1", "/dev/gpiochip2"};
    
    for (const char* chip_name : chip_names) {
        chip_ = gpiod_chip_open(chip_name);
        if (chip_ != nullptr) {
            std::cout << "Jetson GPIO: Successfully opened " << chip_name << std::endl;
            initialized_ = true;
            break;
        }
    }
    
    if (!initialized_) {
        std::cerr << "Jetson GPIO: Failed to open any GPIO chip" << std::endl;
    }
}

JetsonGpio::~JetsonGpio() {
    if (chip_ != nullptr) {
        gpiod_chip_close(chip_);
    }
}

void JetsonGpio::exportPin(int pin) {
    if (!initialized_) {
        std::cerr << "JetsonGpio not initialized" << std::endl;
        return;
    }
    
    // libgpiod에서는 핀을 자동으로 관리하므로 별도의 export가 필요하지 않음
    std::cout << "Jetson GPIO: Pin " << pin << " is ready for use" << std::endl;
}

void JetsonGpio::unexportPin(int pin) {
    if (!initialized_) {
        std::cerr << "JetsonGpio not initialized" << std::endl;
        return;
    }
    
    // libgpiod에서는 핀을 자동으로 관리하므로 별도의 unexport가 필요하지 않음
    std::cout << "Jetson GPIO: Pin " << pin << " released" << std::endl;
}

void JetsonGpio::setDirection(int pin, const std::string& dir) {
    if (!initialized_) {
        std::cerr << "JetsonGpio not initialized" << std::endl;
        return;
    }
    
    std::cout << "Jetson GPIO: Setting pin " << pin << " direction to " << dir << std::endl;
}

void JetsonGpio::writeValue(int pin, int value) {
    if (!initialized_) {
        std::cerr << "JetsonGpio not initialized" << std::endl;
        return;
    }
    
    // libgpiod를 사용하여 실제로 핀에 값을 씁니다
    struct gpiod_line* line = gpiod_chip_get_line(chip_, pin);
    if (line == nullptr) {
        std::cerr << "Jetson GPIO: Failed to get line for pin " << pin << std::endl;
        return;
    }
    
    // 핀이 이미 출력으로 설정되어 있는지 확인
    int ret = gpiod_line_request_output(line, "jetson_gpio", 0);
    if (ret < 0) {
        // 이미 요청된 경우 무시하고 계속 진행
        std::cout << "Jetson GPIO: Pin " << pin << " already configured as output" << std::endl;
    }
    
    ret = gpiod_line_set_value(line, value);
    if (ret < 0) {
        std::cerr << "Jetson GPIO: Failed to set value for pin " << pin << std::endl;
        return;
    }
    
    std::cout << "Jetson GPIO: Successfully wrote " << value << " to pin " << pin << std::endl;
}

int JetsonGpio::readValue(int pin) {
    if (!initialized_) {
        std::cerr << "JetsonGpio not initialized" << std::endl;
        return -1;
    }
    
    // libgpiod를 사용하여 실제로 핀에서 값을 읽습니다
    struct gpiod_line* line = gpiod_chip_get_line(chip_, pin);
    if (line == nullptr) {
        std::cerr << "Jetson GPIO: Failed to get line for pin " << pin << std::endl;
        return -1;
    }
    
    int ret = gpiod_line_request_input(line, "jetson_gpio");
    if (ret < 0) {
        std::cerr << "Jetson GPIO: Failed to request input for pin " << pin << std::endl;
        return -1;
    }
    
    int value = gpiod_line_get_value(line);
    if (value < 0) {
        std::cerr << "Jetson GPIO: Failed to read value from pin " << pin << std::endl;
        return -1;
    }
    
    std::cout << "Jetson GPIO: Successfully read " << value << " from pin " << pin << std::endl;
    return value;
}

bool JetsonGpio::isPinExported(int pin) {
    if (!initialized_) {
        return false;
    }
    
    // libgpiod에서는 핀이 사용 가능한지 확인
    struct gpiod_line* line = gpiod_chip_get_line(chip_, pin);
    return (line != nullptr);
}

std::string JetsonGpio::getGpioPath(int pin) {
    // libgpiod를 사용하므로 sysfs 경로는 사용하지 않음
    return "/dev/gpiochip* (using libgpiod)";
} 