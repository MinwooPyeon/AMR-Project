package com.example.amr_backend.v1.mqtt

import com.example.amr_backend.v1.dto.AmrManualControlMessage
import org.eclipse.paho.client.mqttv3.MqttClient
import org.eclipse.paho.client.mqttv3.MqttMessage
import org.springframework.stereotype.Component

@Component
class MqttPublisher(
    private val mqttClient: MqttClient
) {
    fun sendCommand(message: AmrManualControlMessage) {
        val topic = getManualControlTopic(message.serial)
        mqttClient.publish(topic, MqttMessage(message.operationType.toString().toByteArray()))
    }

    private fun getManualControlTopic(serial: String): String = "control/$serial"
}