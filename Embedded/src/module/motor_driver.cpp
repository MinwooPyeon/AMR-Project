#include "amr/module/motor_driver.h"
#include <stdexcept>

MotorDriver::MotorDriver(std::shared_ptr<GpioInterface> gpio, std::shared_ptr<PwmInterface> pwm,
	int in1Pin, int in2Pin, int pwmChip, int pwmChannel, unsigned int period_ns)
	: gpio_(std::move(gpio)), pwm_(std::move(pwm)), in1Pin_(in1Pin), in2Pin_(in2Pin),
	pwmChip_(pwmChip), pwmChannel_(pwmChannel), period_ns_(period_ns) {

	gpio_->exportPin(in1Pin_);
	gpio_->exportPin(in2Pin_);
	gpio_->setDirection(in1Pin_, "out");
	gpio_->setDirection(in2Pin_, "out");

	pwm_->exportChannel(pwmChip_, pwmChannel_);
	pwm_->setPeriod(pwmChip_, pwmChannel_, period_ns_);
	pwm_->setDutyCycle(pwmChip_, pwmChannel_, 0);
	pwm_->enable(pwmChip_, pwmChannel_, false);

	initialized_ = true;
}

MotorDriver::~MotorDriver() {
	if (initialized_) {
		stop();
		pwm_->enable(pwmChip_, pwmChannel_, false);
		pwm_->unexportChannel(pwmChip_, pwmChannel_);

		gpio_->unexportPin(in1Pin_);
		gpio_->unexportPin(in2Pin_);
	}
}

void MotorDriver::setSpeed(int speed) {
	if (!initialized_) throw std::runtime_error("MotorDriver Not Initialized");

	int s = speed;
	if (s > 100) s = 100;
	else if (s < -100) s = -100;

	if (s > 0) {
		gpio_->writeValue(in1Pin_, 1);
		gpio_->writeValue(in2Pin_, 0);
		unsigned int duty = period_ns_ * s / 100;
		pwm_->setDutyCycle(pwmChip_, pwmChannel_, duty);
		pwm_->enable(pwmChip_, pwmChannel_, true);
	}
	else if (s < 0) {
		gpio_->writeValue(in1Pin_, 0);
		gpio_->writeValue(in2Pin_, 1);
		unsigned int duty = period_ns_ * s / 100;
		pwm_->setDutyCycle(pwmChip_, pwmChannel_, duty);
		pwm_->enable(pwmChip_, pwmChannel_, true);
	}
	else {
		pwm_->enable(pwmChip_, pwmChannel_, false);
		gpio_->writeValue(in1Pin_, 0);
		gpio_->writeValue(in2Pin_, 0);
	}
}

void MotorDriver::stop() {
	setSpeed(0);
}
