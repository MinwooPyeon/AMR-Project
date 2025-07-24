#include "amr/sysfsInterface/sysfs_pwm.h"
#include <fstream>
#include <sys/stat.h>
#include <cstdio>
#include <thread>
#include <chrono>
#include <iostream>


void SysfsPwm::exportChannel(int chip, int channel) {
	// �̹� export �Ǿ� ������ ����
	struct stat st;
	
	if (stat(pwmChannelPath(chip, channel, "enable").c_str(), &st) == 0) return;
	
	std::ofstream f(pwmChipPath(chip) + "/export");
	if (!f) { perror("PWM export"); return; }
	f << channel;
	// sysfs�� ä�� ���͸��� ������ �ð��� ��� ��
	std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

void SysfsPwm::unexportChannel(int chip, int channel) {
	std::ofstream f(pwmChipPath(chip) + "/unexport");
	if (!f) { perror("PWM unexport"); return; }
	f << channel;
}

void SysfsPwm::setPeriod(int chip, int channel, unsigned int period_ns) {
	std::ofstream f(pwmChannelPath(chip, channel, "period"));
	if (!f) { perror("PWM period"); return; }
	f << period_ns;
}

void SysfsPwm::setDutyCycle(int chip, int channel, unsigned int duty_cycle_ns) {
	std::ofstream f(pwmChannelPath(chip, channel, "duty_cycle"));
	if (!f) { perror("PWM duty_cycle"); return; }
	f << duty_cycle_ns;
}

void SysfsPwm::enable(int chip, int channel, bool en) {
	std::ofstream f(pwmChannelPath(chip, channel, "enable"));
	if (!f) { perror("PWM enable"); return; }
	f << (en ? '1' : '0');
}