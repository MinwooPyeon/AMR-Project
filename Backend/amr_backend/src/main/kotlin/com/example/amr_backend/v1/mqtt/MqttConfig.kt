package com.example.amr_backend.v1.mqtt

import org.eclipse.paho.client.mqttv3.MqttClient
import org.eclipse.paho.client.mqttv3.MqttConnectOptions
import org.slf4j.LoggerFactory
import org.springframework.beans.factory.annotation.Value
import org.springframework.context.annotation.Bean
import org.springframework.context.annotation.Configuration

@Configuration
class MqttConfig(
    @Value("\${mqtt.url}") private val mqttUrl: String
) {
    private val logger = LoggerFactory.getLogger(this::class.java)

    @Bean
    fun mqttClient(): MqttClient {
        logger.debug("mqtt connecting to {}", mqttUrl)
        val client = MqttClient(mqttUrl, "spring-server-mqtt", null)
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