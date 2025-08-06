package com.example.amr_backend.v1.mqtt

import com.example.amr_backend.v1.dto.AlertInboundMessage
import com.example.amr_backend.v1.dto.AlertOutboundMessage
import com.example.amr_backend.v1.dto.Situation
import com.example.amr_backend.v1.fcm.FirebaseMessageHandler
import com.example.amr_backend.v1.service.AreaMapper
import com.example.amr_backend.v1.storage.ImageSaver
import com.fasterxml.jackson.databind.ObjectMapper
import jakarta.annotation.PostConstruct
import jakarta.annotation.PreDestroy
import org.eclipse.paho.client.mqttv3.IMqttMessageListener
import org.eclipse.paho.client.mqttv3.MqttClient
import org.eclipse.paho.client.mqttv3.MqttMessage
import org.slf4j.LoggerFactory
import org.springframework.stereotype.Component

private const val ALERT_TOPIC = "alert"

@Component
class AlertMessageHandler(
    private val mqttClient: MqttClient,
    private val firebaseMessageHandler: FirebaseMessageHandler,
    private val imageSaver: ImageSaver,
    private val areaMapper: AreaMapper,
    private val objectMapper: ObjectMapper,
) : IMqttMessageListener {
    private val logger = LoggerFactory.getLogger(this::class.java)

    @PostConstruct
    private fun subscribeTopic() {
        mqttClient.subscribe(ALERT_TOPIC, this)
    }

    @PreDestroy
    private fun unsubscribeTopic() {
        mqttClient.unsubscribe(ALERT_TOPIC)
    }

    override fun messageArrived(topic: String?, message: MqttMessage?) {
        try {
            logger.debug("{} message received : {}", topic, message)
            val inboundMessage = objectMapper.readValue(message?.payload, AlertInboundMessage::class.java)
            val imagePath = imageSaver.save(inboundMessage.image)
            val outboundMessage = inboundMessage.toOutboundMessage(imagePath)
            firebaseMessageHandler.sendAlertMessage(outboundMessage)
        } catch (e: Exception) {
            logger.warn("Failed to handle alert message : {}", e.message)
        }
    }

    private fun AlertInboundMessage.toOutboundMessage(imagePath: String): AlertOutboundMessage {
        val situationMessage = situation.toMessage()
        val area = areaMapper.convertCoordinateToArea(x, y)

        return AlertOutboundMessage(
            title = "$situationMessage 감지",
            summary = "${area}에 ${situationMessage}감지",
            area = area,
            image = imagePath,
        )
    }

    private fun Situation.toMessage() = when (this) {
        Situation.COLLAPSE -> "충돌"
        Situation.SMOKE -> "흡연"
        Situation.EQUIPMENT -> "안전 장비 미착용"
    }
}