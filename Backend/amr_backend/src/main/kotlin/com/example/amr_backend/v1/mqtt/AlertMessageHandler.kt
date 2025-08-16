package com.example.amr_backend.v1.mqtt

import com.example.amr_backend.v1.dto.AlertInboundMessage
import com.example.amr_backend.v1.dto.AlertOutboundMessage
import com.example.amr_backend.v1.dto.Case
import com.example.amr_backend.v1.dto.toEntity
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
import org.springframework.context.ApplicationEventPublisher
import org.springframework.stereotype.Component

private const val ALERT_TOPIC = "alert"

@Component
class AlertMessageHandler(
    private val mqttClient: MqttClient,
    private val firebaseMessageHandler: FirebaseMessageHandler,
    private val imageSaver: ImageSaver,
    private val areaMapper: AreaMapper,
    private val objectMapper: ObjectMapper,
    private val eventPublisher: ApplicationEventPublisher,
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
            eventPublisher.publishEvent(outboundMessage.toEntity())
        } catch (e: Exception) {
            logger.warn("Failed to handle alert message : {}", e.message)
            logger.warn(e.stackTraceToString())
        }
    }

    private fun AlertInboundMessage.toOutboundMessage(imagePath: String): AlertOutboundMessage {
        val caseMessage = case.toMessage()
        val area = areaMapper.convertCoordinateToArea(x, y)

        return AlertOutboundMessage(
            serial = serial,
            title = "$caseMessage 감지",
            content = "${area}에 ${caseMessage}감지",
            area = area,
            case = case,
            image = imagePath,
        )
    }

    private fun Case.toMessage() = when (this) {
        Case.COLLAPSE -> "충돌"
        Case.SMOKE -> "흡연"
        Case.EQUIPMENT -> "안전 장비 미착용"
        Case.FALL -> "사람 쓰러짐"
    }
}