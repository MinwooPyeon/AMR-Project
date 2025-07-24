#ifndef SYSFS_PWM_H
#define SYSFS_PWM_H

#include "amr/sysfsInterface/pwm_interface.h"

/**
 * @brief Sysfs-based implementation of the PwmInterface.
 *
 * Uses the Linux sysfs PWM interface:
 *  - export/unexport:    /sys/class/pwm/pwmchip<chip>/export, unexport
 *  - period:             /sys/class/pwm/pwmchip<chip>/pwm<channel>/period
 *  - duty_cycle:         /sys/class/pwm/pwmchip<chip>/pwm<channel>/duty_cycle
 *  - enable:             /sys/class/pwm/pwmchip<chip>/pwm<channel>/enable
 */
class SysfsPwm : public PwmInterface {
public:
    /**
     * @brief Export a PWM channel via sysfs.
     * @param chip      The PWM chip number.
     * @param channel   The PWM channel number.
     */
    void exportChannel(int chip, int channel) override;

    /**
     * @brief Unexport a PWM channel via sysfs.
     * @param chip      The PWM chip number.
     * @param channel   The PWM channel number.
     */
    void unexportChannel(int chip, int channel) override;

    /**
     * @brief Set the PWM period via sysfs.
     * @param chip       The PWM chip number.
     * @param channel    The PWM channel number.
     * @param period_ns  The period in nanoseconds.
     */
    void setPeriod(int chip, int channel, unsigned int period_ns) override;

    /**
     * @brief Set the PWM duty cycle via sysfs.
     * @param chip            The PWM chip number.
     * @param channel         The PWM channel number.
     * @param duty_cycle_ns   The duty cycle in nanoseconds.
     */
    void setDutyCycle(int chip, int channel, unsigned int duty_cycle_ns) override;

    /**
     * @brief Enable or disable a PWM channel via sysfs.
     * @param chip      The PWM chip number.
     * @param channel   The PWM channel number.
     * @param enable    True to enable, false to disable.
     */
    void enable(int chip, int channel, bool enable) override;

private:
    /**
     * @brief Build the sysfs path to a PWM chip directory.
     * @param chip  The PWM chip number.
     * @return      The path to /sys/class/pwm/pwmchip<chip>.
     */
    std::string pwmChipPath(int chip) const {
        return "/sys/class/pwm/pwmchip" + std::to_string(chip);
    }

    /**
     * @brief Build the sysfs path to a PWM channel file.
     * @param chip      The PWM chip number.
     * @param channel   The PWM channel number.
     * @param file      The filename within the channel directory (e.g., "period").
     * @return          The full path to the specified sysfs file.
     */
    std::string pwmChannelPath(int chip, int channel, const std::string& file) const {
        return "/sys/class/pwm/pwmchip" + std::to_string(chip)
            + "/pwm" + std::to_string(channel) + "/" + file;
    }
};

#endif // SYSFS_PWM_H
