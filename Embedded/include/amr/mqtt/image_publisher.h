#ifndef IMAGE_PUBLISHER_H
#define IMAGE_PUBLISHER_H

#include "amr/mqtt/mqtt_publisher.h"
#include "amr/module/frame.h"
#include <string>

class ImagePublisher : public MqttPublisher
{
public:
    ImagePublisher(const std::string &clientId, const std::string &host, const std::string &topic, int port = 1883, int keepalive = 60);
    bool publish(const Frame &frame);
private:
    std::string topic_;
};
#endif