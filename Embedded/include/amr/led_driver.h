#include "amr/led_driver.h"
#include <stdexcept>

namespace amr {

LedDriver::LedDriver(std::shared_ptr<GpioInterface> gpio,
                     std::shared_ptr<PwmInterface> pwm,
                     Mode mode,
                     int gpioPin, int pwmChip, int pwmChannel)
    : gpio_(gpio), pwm_(pwm), mode_(mode),
      gpioPin_(gpioPin), pwmChip_(pwmChip), pwmChannel_(pwmChannel) {
    if (mode == Mode::GPIO) {
        gpio_->exportPin(gpioPin_);
        gpio_->setDirection(gpioPin_, GpioInterface::Direction::OUT);
    } else {
        pwm_->exportPwm(pwmChip_, pwmChannel_);
        pwm_->setPeriod(pwmChip_, pwmChannel_, 1000000);
        pwm_->enable(pwmChip_, pwmChannel_);
    }
}

void LedDriver::turnOn() {
    if (mode_ == Mode::GPIO) gpio_->setValue(gpioPin_, 1);
    else setBrightness(100);
}

void LedDriver::turnOff() {
    if (mode_ == Mode::GPIO) gpio_->setValue(gpioPin_, 0);
    else setBrightness(0);
}

void LedDriver::setBrightness(int value) {
    if (mode_ != Mode::PWM)
        throw std::runtime_error("Brightness only available in PWM mode");

    if (value < 0 || value > 100)
        throw std::out_of_range("Brightness must be 0 ~ 100");

    auto period = pwm_->getPeriod(pwmChip_, pwmChannel_);
    unsigned int duty = period * value / 100;
    pwm_->setDutyCycle(pwmChip_, pwmChannel_, duty);
}

}
