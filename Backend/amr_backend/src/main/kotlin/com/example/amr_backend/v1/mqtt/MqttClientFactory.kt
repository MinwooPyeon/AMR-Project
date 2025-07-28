package com.example.amr_backend.v1.mqtt

import org.eclipse.paho.client.mqttv3.MqttClient
import org.eclipse.paho.client.mqttv3.MqttConnectOptions
import org.springframework.stereotype.Component

@Component
class MqttClientFactory {
    fun create(url: String): MqttClient {
        val client = MqttClient(url, MqttClient.generateClientId(), null)
        val options = MqttConnectOptions().apply {
            isAutomaticReconnect = true
            isCleanSession = true
        }
        client.connect(options)
        return client
    }
}