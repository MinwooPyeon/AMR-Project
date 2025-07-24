#ifndef SYSFS_GPIO_H
#define SYSFS_GPIO_H

#include <string>
#include "amr/sysfsInterface/gpio_interface.h"

/**
 * @brief Sysfs-based implementation of the GpioInterface.
 *
 * Uses the Linux sysfs GPIO interface:
 *  - export:     /sys/class/gpio/export
 *  - unexport:   /sys/class/gpio/unexport
 *  - direction:  /sys/class/gpio/gpio<pin>/direction
 *  - value:      /sys/class/gpio/gpio<pin>/value
 */
class SysfsGpio : public GpioInterface {
public:
    /**
     * @brief Export a GPIO pin via sysfs.
     * @param pin  The BCM pin number.
     */
    void exportPin(int pin) override;

    /**
     * @brief Unexport a GPIO pin via sysfs.
     * @param pin  The BCM pin number.
     */
    void unexportPin(int pin) override;

    /**
     * @brief Set the direction of a GPIO pin via sysfs.
     * @param pin  The BCM pin number.
     * @param dir  The direction ("in" or "out").
     */
    void setDirection(int pin, const std::string& dir) override;

    /**
     * @brief Write a value to a GPIO pin via sysfs.
     * @param pin    The BCM pin number.
     * @param value  The value to write (0 or 1).
     */
    void writeValue(int pin, int value) override;

    /**
     * @brief Read the value of a GPIO pin via sysfs.
     * @param pin  The BCM pin number.
     * @return     The current GPIO value (0 or 1).
     */
    int readValue(int pin) override;

private:
    /**
     * @brief Build the sysfs path to a GPIO file.
     * @param pin   The BCM pin number.
     * @param file  The filename within the pin directory (e.g., "value", "direction").
     * @return      The full path to the specified sysfs file.
     */
    std::string gpioPath(int pin, const std::string& file) const {
        return "/sys/class/gpio/gpio" + std::to_string(pin) + "/" + file;
    }
};

#endif // SYSFS_GPIO_H
