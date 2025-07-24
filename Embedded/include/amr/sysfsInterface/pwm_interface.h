#ifndef PWM_INTERFACE_H
#define PWM_INTERFACE_H

#include <string>

/**
 * @brief Pure virtual interface for PWM control.
 * 
 * Provides methods for exporting/unexporting PWM Channel,
 * setting PWM period, duty cycle and enable/disable.
 */
class PwmInterface {
public:
    virtual ~PwmInterface() = default;

    /**
     * @brief Export a PWM channel.
     * @param chip      The PWM chip number.
     * @param channel   The PWM channel number.
     */
    virtual void exportChannel(int chip, int channel) = 0;

    /**
     * @brief Unexport a PWM channel.
     * @param chip      The PWM chip number.
     * @param channel   The PWM channel number.
     */
    virtual void unexportChannel(int chip, int channel) = 0;

    /**
     * @brief Set the PWM period.
     * @param chip       The PWM chip number.
     * @param channel    The PWM channel number.
     * @param period_ns  The period in nanoseconds.
     */
    virtual void setPeriod(int chip, int channel, unsigned int period_ns) = 0;

    /**
     * @brief Set the PWM duty cycle.
     * @param chip            The PWM chip number.
     * @param channel         The PWM channel number.
     * @param duty_cycle_ns   The duty cycle in nanoseconds.
     */
    virtual void setDutyCycle(int chip, int channel, unsigned int duty_cycle_ns) = 0;

    /**
     * @brief Enable or disable the PWM channel.
     * @param chip      The PWM chip number.
     * @param channel   The PWM channel number.
     * @param enable    True to enable, false to disable.
     */
    virtual void enable(int chip, int channel, bool enable) = 0;
};

#endif // PWM_INTERFACE_H
