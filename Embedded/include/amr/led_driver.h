#ifndef LED_DRIVER_H
#define LED_DRIVER_H

#include "amr/sysfsInterface/gpio_interface.h"
#include "amr/sysfsInterface/pwm_interface.h"
#include <memory>

namespace amr {

enum class Mode {
    GPIO,
    PWM
};

class LedDriver {
public:
    LedDriver(std::shared_ptr<GpioInterface> gpio,
              std::shared_ptr<PwmInterface> pwm,
              Mode mode,
              int gpioPin, int pwmChip, int pwmChannel);
    
    void turnOn();
    void turnOff();
    void setBrightness(int value);

private:
    std::shared_ptr<GpioInterface> gpio_;
    std::shared_ptr<PwmInterface> pwm_;
    Mode mode_;
    int gpioPin_;
    int pwmChip_;
    int pwmChannel_;
};

}

#endif
