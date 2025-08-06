#include "amr/mqtt/state_publisher.h"
#include <ctime>
#include <iostream>

StatePublisher::StatePublisher(const std::string &clientId, const std::string &host, const std::string &topic, int port, int keepalive)
    : MqttPublisher(clientId, host, port, keepalive), topic_(topic) {}

bool StatePublisher::publish(const std::string &jsonData)
{
    if (!connected_)
    {
        std::cerr << "[MetadataPublisher] Not connected\n";
        return false;
    }

    int rc = mosquitto_publish(mosq_, nullptr, topic_.c_str(), jsonData.size(), jsonData.c_str(), 0, false);
    return rc == MOSQ_ERR_SUCCESS;
}