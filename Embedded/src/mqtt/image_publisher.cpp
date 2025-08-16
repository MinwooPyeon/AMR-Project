#include "amr/mqtt/image_publisher.h"
#include <iostream>
#include <mosquitto.h>

ImagePublisher::ImagePublisher(const std::string &clientId, const std::string &host, const std::string &topic, int port, int keepalive)
    : MqttPublisher(clientId, host, port, keepalive), topic_(topic) {}

bool ImagePublisher::publish(const Frame &frame)
{
    if (!connected_)
    {
        std::cerr << "[ImagePublisher] Not connected\n";
        return false;
    }

    int rc = mosquitto_publish(mosq_, nullptr, topic_.c_str(), frame.size(), frame.data(), 0, false);
    return rc == MOSQ_ERR_SUCCESS;
}
