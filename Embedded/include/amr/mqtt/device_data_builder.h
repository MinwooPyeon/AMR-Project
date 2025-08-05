#ifndef DEVICE_DATA_BUILDER_H
#define DEVICE_DATA_BUILDER_H

#include <string>
class DeviceDataBuilder
{
public:
    static std::string build(const std::string &deviceId, const std::string &status);
};

#endif