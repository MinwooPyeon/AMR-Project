#include "amr/sysfsInterface/sysfs_gpio.h"
#include <fstream>
#include <sys/stat.h>
#include <cstdio>
#include <iostream>

void SysfsGpio::exportPin(int pin) {
    // �̹� export �Ǿ� ������ ����
    struct stat st;
    if (stat(gpioPath(pin, "value").c_str(), &st) == 0) return;

    std::ofstream f("/sys/class/gpio/export");
    if (!f) { perror("export"); return; }
    f << pin;
}

void SysfsGpio::unexportPin(int pin) {
    std::ofstream f("/sys/class/gpio/unexport");
    if (!f) { perror("unexport"); return; }
    f << pin;
}

void SysfsGpio::setDirection(int pin, const std::string& dir) {
    std::ofstream f(gpioPath(pin, "direction"));
    if (!f) { perror("direction"); return; }
    f << dir;
}

void SysfsGpio::writeValue(int pin, int value) {
    std::ofstream f(gpioPath(pin, "value"));
    if (!f) { perror("value"); return; }
    f << (value ? '1' : '0');
}

int SysfsGpio::readValue(int pin) {
    std::ifstream f(gpioPath(pin, "value"));
    if (!f) { perror("value"); return -1; }
    char c = '0';
    f >> c;
    return (c == '0') ? 0 : 1;
}
