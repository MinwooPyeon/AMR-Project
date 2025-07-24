#pragma once

namespace amr {

class GpioInterface {
public:
    enum class Direction { IN, OUT };

    virtual ~GpioInterface() = default;

    virtual void exportPin(int pin) = 0;
    virtual void unexportPin(int pin) = 0;
    virtual void setDirection(int pin, Direction dir) = 0;
    virtual void setValue(int pin, int value) = 0;
    virtual int getValue(int pin) = 0;
};

}  