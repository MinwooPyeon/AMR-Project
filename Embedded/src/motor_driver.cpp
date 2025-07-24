#include "amr/motor_driver.h"
#include <cmath>
#include <stdexcept>

namespace amr {

MotorDriver::MotorDriver(std::shared_ptr<GpioInterface> gpio,
                         std::shared_ptr<PwmInterface> pwm,
                         int dirPin1, int dirPin2,
                         int pwmChip, int pwmChannel)
    : gpio_(gpio), pwm_(pwm),
      dirPin1_(dirPin1), dirPin2_(dirPin2),
      pwmChip_(pwmChip), pwmChannel_(pwmChannel) {
    gpio_->exportPin(dirPin1_);
    gpio_->exportPin(dirPin2_);
    gpio_->setDirection(dirPin1_, GpioInterface::Direction::OUT);
    gpio_->setDirection(dirPin2_, GpioInterface::Direction::OUT);
    pwm_->exportPwm(pwmChip_, pwmChannel_);
    pwm_->setPeriod(pwmChip_, pwmChannel_, 1000000);
    pwm_->enable(pwmChip_, pwmChannel_);
}

void MotorDriver::setSpeed(int speed) {
    if (speed < -100 || speed > 100) {
        throw std::out_of_range("Speed must be between -100 and 100");
    }

    if (speed > 0) {
        gpio_->setValue(dirPin1_, 1);
        gpio_->setValue(dirPin2_, 0);
    } else if (speed < 0) {
        gpio_->setValue(dirPin1_, 0);
        gpio_->setValue(dirPin2_, 1);
    } else {
        stop();
        return;
    }

    auto absSpeed = std::abs(speed);
    auto period = pwm_->getPeriod(pwmChip_, pwmChannel_);
    auto duty = period * absSpeed / 100;
    pwm_->setDutyCycle(pwmChip_, pwmChannel_, duty);
}

void MotorDriver::stop() {
    gpio_->setValue(dirPin1_, 0);
    gpio_->setValue(dirPin2_, 0);
    pwm_->setDutyCycle(pwmChip_, pwmChannel_, 0);
}

}
