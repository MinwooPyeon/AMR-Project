#include "amr/mqtt/robot_data_publisher.h"
#include <ctime>
#include <iostream>
#include <nlohmann/json.hpp>

RobotDataPublisher::RobotDataPublisher(const std::string &clientId, const std::string &host, const std::string &topic, int port, int keepalive)
    : MqttPublisher(clientId, host, port, keepalive), topic_(topic) {}

bool RobotDataPublisher::publishRobotData(const std::string &serial, 
                                         const std::string &state, 
                                         float x, 
                                         float y, 
                                         float speed, 
                                         float angle)
{
    if (!connected_)
    {
        std::cerr << "[RobotDataPublisher] Not connected\n";
        return false;
    }

    nlohmann::json j;
    j["serial"] = serial;
    j["state"] = state;
    j["x"] = x;
    j["y"] = y;
    j["speed"] = speed;
    j["angle"] = angle;

    std::string jsonData = j.dump();
    
    int rc = mosquitto_publish(mosq_, nullptr, topic_.c_str(), jsonData.size(), jsonData.c_str(), 0, false);
    
    if (rc == MOSQ_ERR_SUCCESS) {
        std::cout << "[RobotDataPublisher] Published robot data: " << jsonData << std::endl;
        return true;
    } else {
        std::cerr << "[RobotDataPublisher] Failed to publish robot data" << std::endl;
        return false;
    }
}

bool RobotDataPublisher::publish(const std::string &jsonData)
{
    if (!connected_)
    {
        std::cerr << "[RobotDataPublisher] Not connected\n";
        return false;
    }

    int rc = mosquitto_publish(mosq_, nullptr, topic_.c_str(), jsonData.size(), jsonData.c_str(), 0, false);
    return rc == MOSQ_ERR_SUCCESS;
} 