#ifndef JETSON_GPIO_H
#define JETSON_GPIO_H

#include "amr/sysfsInterface/gpio_interface.h"
#include <string>
#include <memory>

struct gpiod_chip;

class JetsonGpio : public GpioInterface {
public:
    JetsonGpio();
    ~JetsonGpio();

    void exportPin(int pin) override;
    void unexportPin(int pin) override;
    void setDirection(int pin, const std::string& dir) override;
    void writeValue(int pin, int value) override;
    int readValue(int pin) override;

private:
    bool isPinExported(int pin);
    std::string getGpioPath(int pin);
    
    struct gpiod_chip* chip_;
    bool initialized_;
};

#endif 