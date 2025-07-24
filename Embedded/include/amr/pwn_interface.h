#pragma once

namespace amr {

class PwmInterface {
public:
    virtual ~PwmInterface() = default;

    virtual void exportPwm(int chip, int channel) = 0;
    virtual void unexportPwm(int chip, int channel) = 0;
    virtual void setPeriod(int chip, int channel, unsigned int period_ns) = 0;
    virtual unsigned int getPeriod(int chip, int channel) = 0;
    virtual void setDutyCycle(int chip, int channel, unsigned int duty_ns) = 0;
    virtual void enable(int chip, int channel) = 0;
    virtual void disable(int chip, int channel) = 0;
};

} 