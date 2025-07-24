#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include "amr/sysfsInterface/gpio_interface.h"
#include "amr/sysfsInterface/pwm_interface.h"
#include <memory>

/**
 * @brief Driver for a single motor using GPIO for direction and PWM for speed control.
 *
 * This class controls motor direction via two GPIO pins (IN1, IN2)
 * and motor speed via a PWM channel.
 */
class MotorDriver {
public:
    /**
     * @brief Construct a new MotorDriver.
     *
     * Initializes the motor driver with the given GPIO and PWM interfaces,
     * direction pins, and PWM configuration.
     *
     * @param gpio         Shared pointer to a GPIO interface implementation.
     * @param pwm          Shared pointer to a PWM interface implementation.
     * @param in1Pin       BCM pin number for IN1 (forward direction).
     * @param in2Pin       BCM pin number for IN2 (reverse direction).
     * @param pwmChip      PWM chip number.
     * @param pwmChannel   PWM channel number.
     * @param period_ns    PWM period in nanoseconds.
     */
    MotorDriver(std::shared_ptr<GpioInterface> gpio,
        std::shared_ptr<PwmInterface> pwm,
        int in1Pin,
        int in2Pin,
        int pwmChip,
        int pwmChannel,
        unsigned int period_ns);

    /**
     * @brief Destroy the MotorDriver.
     *
     * Stops the motor and releases resources.
     */
    ~MotorDriver();

    /**
     * @brief Set the motor speed.
     *
     * Positive values drive the motor forward, negative values drive it in reverse,
     * and zero stops the motor.
     *
     * @param speed  The desired motor speed.
     */
    void setSpeed(int speed);

    /**
     * @brief Stop the motor immediately.
     *
     * Disables PWM output and sets both direction pins low.
     */
    void stop();

private:
    std::shared_ptr<GpioInterface> gpio_;   ///< GPIO interface implementation
    std::shared_ptr<PwmInterface> pwm_;     ///< PWM interface implementation
    int in1Pin_;                            ///< BCM pin for IN1
    int in2Pin_;                            ///< BCM pin for IN2
    int pwmChip_;                           ///< PWM chip number
    int pwmChannel_;                        ///< PWM channel number
    unsigned int period_ns_;                ///< PWM period in nanoseconds
    bool initialized_ = false;              ///< Indicates if the driver has been initialized
};

#endif // MOTOR_DRIVER_H
