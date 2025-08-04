package com.example.amr_backend.v1.mqtt

import com.example.amr_backend.v1.dto.AmrManualControlMessage
import com.fasterxml.jackson.databind.ObjectMapper
import org.eclipse.paho.client.mqttv3.MqttClient
import org.eclipse.paho.client.mqttv3.MqttMessage
import org.springframework.stereotype.Component

@Component
class MqttPublisher(
    private val mqttClient: MqttClient,
    private val objectMapper: ObjectMapper,
) {
    fun sendCommand(serial: String, message: AmrManualControlMessage) {
        val topic = getManualControlTopic(serial)
        val jsonString = objectMapper.writeValueAsString(message)
        mqttClient.publish(topic, MqttMessage(jsonString.toByteArray()))
    }

    private fun getManualControlTopic(serial: String): String = "control/$serial"
}