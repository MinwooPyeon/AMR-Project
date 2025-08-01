package com.example.amr_backend.v1.mqtt

import org.eclipse.paho.client.mqttv3.MqttClient
import org.eclipse.paho.client.mqttv3.MqttConnectOptions
import org.springframework.context.annotation.Bean
import org.springframework.context.annotation.Configuration

@Configuration
class MqttConfig {
    @Bean
    fun mqttClient(): MqttClient {
        val client = MqttClient("tcp://localhost:1883", "spring-server-mqtt", null)
        val connectionOptions = MqttConnectOptions().apply {
            isAutomaticReconnect = true
            isCleanSession = true
            connectionTimeout = 10
            keepAliveInterval = 60
        }
        client.connect(connectionOptions)

        return client
    }
}