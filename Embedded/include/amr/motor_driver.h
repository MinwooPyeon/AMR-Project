#pragma once
#include <memory>
#include "gpio_interface.h"
#include "pwm_interface.h"

namespace amr {

class MotorDriver {
public:
    MotorDriver(std::shared_ptr<GpioInterface> gpio,
                std::shared_ptr<PwmInterface> pwm,
                int dirPin1, int dirPin2,
                int pwmChip, int pwmChannel);

    void setSpeed(int speed); 
    void stop();

private:
    std::shared_ptr<GpioInterface> gpio_;
    std::shared_ptr<PwmInterface> pwm_;
    int dirPin1_, dirPin2_;
    int pwmChip_, pwmChannel_;
};

}
