package com.example.amr_backend.v1.mqtt

import com.example.amr_backend.v1.entity.AmrStatus
import com.example.amr_backend.v1.repository.AmrStatusRepository
import com.fasterxml.jackson.databind.ObjectMapper
import org.eclipse.paho.client.mqttv3.IMqttMessageListener
import org.eclipse.paho.client.mqttv3.MqttMessage
import org.slf4j.LoggerFactory
import org.springframework.stereotype.Component
import org.springframework.transaction.annotation.Transactional

@Component
class MqttMessageHandler(
    private val amrStatusRepository: AmrStatusRepository,
    private val objectMapper: ObjectMapper,
) : IMqttMessageListener {
    private val logger = LoggerFactory.getLogger(this::class.java)

    @Transactional
    override fun messageArrived(topic: String?, message: MqttMessage?) {
        try {
            val amrStatus = objectMapper.readValue(message?.payload, AmrStatus::class.java)
            amrStatusRepository.save(amrStatus)
        } catch (e: Exception) {
            logger.warn("Failed to handle a message : {}", e.message)
        }
    }
}