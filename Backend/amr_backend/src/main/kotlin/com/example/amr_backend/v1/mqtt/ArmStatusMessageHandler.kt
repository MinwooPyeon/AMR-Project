package com.example.amr_backend.v1.mqtt

import com.example.amr_backend.v1.dto.AmrStatusMessage
import com.example.amr_backend.v1.dto.toAmrStatus
import com.example.amr_backend.v1.repository.AmrRepository
import com.example.amr_backend.v1.repository.AmrStatusRepository
import com.fasterxml.jackson.databind.ObjectMapper
import jakarta.annotation.PostConstruct
import jakarta.annotation.PreDestroy
import org.eclipse.paho.client.mqttv3.IMqttMessageListener
import org.eclipse.paho.client.mqttv3.MqttClient
import org.eclipse.paho.client.mqttv3.MqttMessage
import org.slf4j.LoggerFactory
import org.springframework.stereotype.Component
import org.springframework.transaction.annotation.Transactional

private const val STATUS_TOPIC = "status"

@Component
class ArmStatusMessageHandler(
    private val amrStatusRepository: AmrStatusRepository,
    private val mqttClient: MqttClient,
    private val amrRepository: AmrRepository,
    private val objectMapper: ObjectMapper,
) : IMqttMessageListener {
    private val logger = LoggerFactory.getLogger(this::class.java)

    @PostConstruct
    fun subscribeStatusTopic() {
        mqttClient.subscribe(STATUS_TOPIC, this)
    }

    @PreDestroy
    fun unsubscribeStatusTopic() {
        mqttClient.unsubscribe(STATUS_TOPIC)
    }

    @Transactional
    override fun messageArrived(topic: String?, message: MqttMessage?) {
        try {
            logger.debug("{} message received : {}", topic, message)
            val amrStatusMessage = objectMapper.readValue(message?.payload, AmrStatusMessage::class.java)
            val amr = amrRepository.findBySerial(amrStatusMessage.serial)
            amrStatusRepository.save(amrStatusMessage.toAmrStatus(amr))
        } catch (e: Exception) {
            logger.warn("Failed to handle a message : {}", e.message)
        }
    }
}