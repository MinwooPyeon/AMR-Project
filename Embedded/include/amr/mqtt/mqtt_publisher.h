#ifndef MQTT_PUBLISHER_H
#define MQTT_PUBLISHER_H

#include <mosquitto.h>
#include <string>
#include <memory>

class MqttPublisher
{
public:
    MqttPublisher(const std::string &clientId, const std::string &host, int port = 1883, int keepalive = 60);
    ~MqttPublisher();

    bool connect();
    void disconnect();

protected:
    std::string clientId_;
    std::string host_;
    int port_;
    int keepalive_;
    struct mosquitto *mosq_;
    bool connected_;
};

#endif