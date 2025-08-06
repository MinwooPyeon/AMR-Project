#include "amr/mqtt/device_data_builder.h"
#include <nlohmann/json.hpp>
#include <ctime>

std::string DeviceDataBuilder::build(const std::string &deviceId,
                                     const std::string &status)
{
    nlohmann::json j;
    j["device_id"] = deviceId;
    j["timestamp"] = std::time(nullptr);
    j["status"] = status;
    return j.dump();
}