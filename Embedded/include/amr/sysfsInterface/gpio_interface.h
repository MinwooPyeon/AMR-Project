#ifndef GPIO_INTERFACE_H
#define GPIO_INTERFACE_H

#include <string>

/**
 * @brief Pure virtual interface for GPIO control.
 *
 * Provides methods for exporting/unexporting GPIO pins,
 * setting pin direction, and reading/writing pin values.
 */
class GpioInterface {
public:
    enum class Direction { IN, OUT };

    virtual ~GpioInterface() = default;

    /**
     * @brief Export a GPIO pin.
     * @param pin  The BCM pin number to export.
     */
    virtual void exportPin(int pin) = 0;

    /**
     * @brief Unexport a GPIO pin.
     * @param pin  The BCM pin number to unexport.
     */
    virtual void unexportPin(int pin) = 0;

    /**
     * @brief Set the direction of a GPIO pin.
     * @param pin  The BCM pin number.
     * @param dir  The direction string ("in" or "out").
     */
    virtual void setDirection(int pin, const std::string& dir) = 0;

    /**
     * @brief Write a value to a GPIO pin.
     * @param pin    The BCM pin number.
     * @param value  The value to write (0 or 1).
     */
    virtual void writeValue(int pin, int value) = 0;

    /**
     * @brief Read the current value of a GPIO pin.
     * @param pin  The BCM pin number.
     * @return     The read value (0 or 1).
     */
    virtual int readValue(int pin) = 0;
};
#endif // GPIO_INTERFACE_H
