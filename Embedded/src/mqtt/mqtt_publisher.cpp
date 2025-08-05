#include "amr/mqtt/mqtt_publisher.h"
#include <iostream>

MqttPublisher::MqttPublisher(const std::string &clientId, const std::string &host, int port, int keepalive)
    : clientId_(clientId), host_(host), port_(port), keepalive_(keepalive), connected_(false)
{
    mosquitto_lib_init();
    mosq_ = mosquitto_new(clientId_.c_str(), true, nullptr);
    if (!mosq_)
        std::cerr << "[MQTT] Failed to create Client\n";
}

MqttPublisher::~MqttPublisher()
{
    if (connected_)
        disconnect();
    if (mosq_)
        mosquitto_destroy(mosq_);
    mosquitto_lib_cleanup();
}

bool MqttPublisher::connect()
{
    int rc = mosquitto_connect(mosq_, host_.c_str(), port_, keepalive_);
    if (rc != MOSQ_ERR_SUCCESS)
    {
        std::cerr << "[MQTT] Connect failed: " << mosquitto_strerror(rc) << "\n";
        return false;
    }
    connected_ = true;
    return true;
}

void MqttPublisher::disconnect()
{
    mosquitto_disconnect(mosq_);
    connected_ = false;
}