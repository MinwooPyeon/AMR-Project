#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <memory>
#include "amr/module/motor_driver.h"

/**
 * @brief Controller for two motors (left and right).
 *
 * Provides operations to move straight and rotate the robot in place.
 */
class MotorController {
public:
    /**
     * @brief Construct a new MotorController.
     * @param leftMotor   Shared pointer to the MotorDriver controlling the left motor.
     * @param rightMotor  Shared pointer to the MotorDriver controlling the right motor.
     */
    MotorController(std::shared_ptr<MotorDriver> leftMotor,
        std::shared_ptr<MotorDriver> rightMotor);

    /**
     * @brief Move the robot straight forward or backward.
     * @param speed  Speed at which to move (positive: forward, negative: backward, zero: stop).
     */
    void moveStraight(int speed);

    /**
     * @brief Rotate the robot to the right in place.
     * @param speed  Rotation speed (positive: clockwise, zero: stop).
     */
    void rotateRight(int speed);

    /**
     * @brief Rotate the robot to the left in place.
     * @param speed  Rotation speed (positive: counterclockwise, zero: stop).
     */
    void rotateLeft(int speed);

    /**
    * @brief Stop the Robot
    */
    void stop();
private:
    std::shared_ptr<MotorDriver> leftMotor_;  ///< MotorDriver for the left motor.
    std::shared_ptr<MotorDriver> rightMotor_; ///< MotorDriver for the right motor.
};

#endif // MOTOR_CONTROLLER_H
