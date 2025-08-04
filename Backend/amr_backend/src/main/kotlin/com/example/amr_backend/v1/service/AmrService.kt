package com.example.amr_backend.v1.service

import com.example.amr_backend.v1.dto.AmrManualControlMessage
import com.example.amr_backend.v1.entity.AmrStatus
import com.example.amr_backend.v1.mqtt.MqttMessageHandler
import com.example.amr_backend.v1.mqtt.MqttPublisher
import com.example.amr_backend.v1.repository.AmrStatusRepository
import com.example.amr_backend.v1.repository.findAmrStatusById
import jakarta.annotation.PostConstruct
import jakarta.annotation.PreDestroy
import org.eclipse.paho.client.mqttv3.MqttClient
import org.springframework.stereotype.Service

private const val STATUS_TOPIC = "status"

@Service
class AmrService(
    private val amrStatusRepository: AmrStatusRepository,
    private val mqttClient: MqttClient,
    private val mqttMessageHandler: MqttMessageHandler,
    private val mqttPublisher: MqttPublisher,
) {
    @PostConstruct
    fun subscribeStatusTopic() {
        mqttClient.subscribe(STATUS_TOPIC, mqttMessageHandler)
    }

    @PreDestroy
    fun unsubscribeStatusTopic() {
        mqttClient.unsubscribe(STATUS_TOPIC)
    }

    fun findAllLatestStatuses(): List<AmrStatus> = amrStatusRepository.findAllLatestStatuses()

    fun findAmrDetail(id: Long): AmrStatus = amrStatusRepository.findAmrStatusById(id)

    fun sendManualControlMessage(serial: String, message: AmrManualControlMessage) =
        mqttPublisher.sendCommand(serial, message)
}