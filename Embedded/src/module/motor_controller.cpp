#include "amr/module/motor_controller.h"

MotorController::MotorController(std::shared_ptr<MotorDriver> leftMotor, std::shared_ptr<MotorDriver> rightMotor)
	: leftMotor_(std::move(leftMotor)), rightMotor_(std::move(rightMotor)) {
	leftMotor_->stop();
	rightMotor_->stop();
}

void MotorController::moveStraight(int speed) {
	leftMotor_->setSpeed(speed);
	rightMotor_->setSpeed(speed);
}

void MotorController::rotateRight(int speed) {
	leftMotor_->setSpeed(speed);
	rightMotor_->setSpeed(speed * -1);
}

void MotorController::rotateLeft(int speed) {
	leftMotor_->setSpeed(speed * -1);
	rightMotor_->setSpeed(speed);
}

void MotorController::stop() {
	leftMotor_->setSpeed(0);
	rightMotor_->setSpeed(0);
}