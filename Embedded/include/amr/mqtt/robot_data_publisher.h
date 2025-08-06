#ifndef ROBOT_DATA_PUBLISHER_H
#define ROBOT_DATA_PUBLISHER_H

#include "amr/mqtt/mqtt_publisher.h"
#include <string>
#include <nlohmann/json.hpp>

class RobotDataPublisher : public MqttPublisher
{
public:
    RobotDataPublisher(const std::string &clientId, const std::string &host, const std::string &topic, int port = 1883, int keepalive = 60);
    
    // 임베디드에서 보내는 데이터 구조에 맞는 publish 메서드
    bool publishRobotData(const std::string &serial, 
                         const std::string &state, 
                         float x, 
                         float y, 
                         float speed, 
                         float angle);
    
    // JSON 문자열로 직접 publish
    bool publish(const std::string &jsonData);

private:
    const std::string topic_;
};

#endif 