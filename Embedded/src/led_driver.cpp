#include "amr/led_driver.h"
#include <stdexcept>
#include <cmath>   

namespace amr {

LedDriver::LedDriver(std::shared_ptr<GpioInterface> gpio,
                     std::shared_ptr<PwmInterface> pwm,
                     Mode mode, int gpioPin, int pwmChip, int pwmChannel)
    : gpio_(gpio), pwm_(pwm), mode_(mode),
      gpioPin_(gpioPin), pwmChip_(pwmChip), pwmChannel_(pwmChannel)
{
    if (mode_ == Mode::GPIO) {
        if (!gpio_) {
            throw std::invalid_argument("GPIO interface is null for GPIO mode");
        }
        gpio_->exportPin(gpioPin_);
        gpio_->setDirection(gpioPin_, "out");
    } 
    else if (mode_ == Mode::PWM) {
        if (!pwm_) {
            throw std::invalid_argument("PWM interface is null for PWM mode");
        }
        pwm_->exportChannel(pwmChip_, pwmChannel_);
        pwm_->setPeriod(pwmChip_, pwmChannel_, 1000000); // 1ms 주기
        pwm_->enable(pwmChip_, pwmChannel_, true);
    } 
    else {
        throw std::invalid_argument("Unknown LED mode specified");
    }
}


void LedDriver::turnOn() {
    if (mode_ == Mode::GPIO) {
        if (!gpio_) throw std::runtime_error("GPIO interface not initialized");
        gpio_->writeValue(gpioPin_, 1);
    } 
    else if (mode_ == Mode::PWM) {
        setBrightness(100);
    }
}


void LedDriver::turnOff() {
    if (mode_ == Mode::GPIO) {
        if (!gpio_) throw std::runtime_error("GPIO interface not initialized");
        gpio_->writeValue(gpioPin_, 0);
    } 
    else if (mode_ == Mode::PWM) {
        setBrightness(0);
    }
}


void LedDriver::setBrightness(int value) {
    if (mode_ != Mode::PWM) {
        throw std::runtime_error("setBrightness() is only available in PWM mode.");
    }
    if (!pwm_) {
        throw std::runtime_error("PWM interface is null in setBrightness");
    }
    if (value < 0 || value > 100) {
        throw std::out_of_range("Brightness must be in range [0 - 100]");
    }

    // 고정된 주기값 사용 (1ms = 1000000ns)
    unsigned int period = 1000000;
    unsigned int duty = static_cast<unsigned int>(std::round(period * value / 100.0));
    pwm_->setDutyCycle(pwmChip_, pwmChannel_, duty);
}

}  
