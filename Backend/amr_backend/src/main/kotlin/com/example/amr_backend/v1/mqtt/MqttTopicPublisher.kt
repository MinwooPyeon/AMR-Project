package com.example.amr_backend.v1.mqtt

import com.example.amr_backend.v1.dto.TopicMessage
import com.example.amr_backend.v1.service.TopicPublisher
import com.fasterxml.jackson.databind.ObjectMapper
import org.eclipse.paho.client.mqttv3.MqttClient
import org.eclipse.paho.client.mqttv3.MqttMessage
import org.springframework.stereotype.Component

@Component
class MqttTopicPublisher(
    private val mqttClient: MqttClient,
    private val objectMapper: ObjectMapper,
) : TopicPublisher {
    override fun publish(topic: String, message: TopicMessage) = when (message) {
        is TopicMessage.AmrManualControlMessage -> sendManualControlMessage(topic, message)
    }

    private fun sendManualControlMessage(topic: String, message: TopicMessage.AmrManualControlMessage) {
        val jsonString = objectMapper.writeValueAsString(message)
        mqttClient.publish(topic, MqttMessage(jsonString.toByteArray()))
    }
}