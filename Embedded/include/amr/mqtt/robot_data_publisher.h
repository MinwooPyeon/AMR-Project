#ifndef ROBOT_DATA_PUBLISHER_H
#define ROBOT_DATA_PUBLISHER_H

#include "amr/mqtt/mqtt_publisher.h"
#include <string>
#include <nlohmann/json.hpp>

class RobotDataPublisher : public MqttPublisher
{
public:
    RobotDataPublisher(const std::string &clientId, const std::string &host, const std::string &topic, int port = 1883, int keepalive = 60);
    
    bool publishRobotData(const std::string &serial, 
                         const std::string &state, 
                         float x, 
                         float y, 
                         float speed, 
                         float angle);
    
    bool publish(const std::string &jsonData);

private:
    const std::string topic_;
};

#endif 