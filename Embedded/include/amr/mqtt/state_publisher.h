#ifndef STATE_PUBLISHER_H
#define STATE_PUBLISHER_H

#include "amr/mqtt/mqtt_publisher.h"
#include <string>

class StatePublisher : public MqttPublisher
{
public:
    StatePublisher(const std::string &clientId, const std::string &host, const std::string &topic, int port = 1883, int keepalive = 60);
    bool publish(const std::string &jsonData);

private:
    const std::string topic_;
};

#endif