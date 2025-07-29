package com.example.amr_backend.v1.mqtt

import com.example.amr_backend.v1.entity.AmrStatus
import com.example.amr_backend.v1.repository.AmrStatusRepository
import com.fasterxml.jackson.databind.ObjectMapper
import org.eclipse.paho.client.mqttv3.IMqttMessageListener
import org.eclipse.paho.client.mqttv3.MqttMessage
import org.springframework.stereotype.Component
import org.springframework.transaction.annotation.Transactional

@Component
class MqttMessageHandler(
    private val amrStatusRepository: AmrStatusRepository,
    private val objectMapper: ObjectMapper,
) : IMqttMessageListener {
    @Transactional
    override fun messageArrived(topic: String?, message: MqttMessage?) {
        val payloadContents = if (message != null) String(message.payload) else null
        val amrStatus = objectMapper.readValue(payloadContents, AmrStatus::class.java)
        amrStatusRepository.save(amrStatus)
    }
}