package com.example.amr_backend.v1.service

import com.example.amr_backend.v1.dto.AmrManualControlMessage
import com.example.amr_backend.v1.entity.AmrStatus
import com.example.amr_backend.v1.mqtt.MqttMessageHandler
import com.example.amr_backend.v1.mqtt.MqttPublisher
import com.example.amr_backend.v1.repository.AmrRepository
import com.example.amr_backend.v1.repository.AmrStatusRepository
import com.example.amr_backend.v1.repository.findAmrStatusById
import jakarta.annotation.PostConstruct
import jakarta.annotation.PreDestroy
import org.eclipse.paho.client.mqttv3.MqttClient
import org.springframework.stereotype.Service

@Service
class AmrService(
    private val amrStatusRepository: AmrStatusRepository,
    private val mqttClient: MqttClient,
    private val mqttMessageHandler: MqttMessageHandler,
    private val amrRepository: AmrRepository,
    private val mqttPublisher: MqttPublisher,
) {
    private val subscribedTopics = mutableSetOf<String>()

    @PostConstruct
    fun subscribeAllAmrs() {
        val amrSerials = amrRepository.findAll().map { it.serial }

        for (amrSerial in amrSerials) {
            val topic = "status/$amrSerial"
            mqttClient.subscribe(topic, mqttMessageHandler)
            subscribedTopics.add(topic)
        }
    }

    @PreDestroy
    fun unsubscribeAllAmrs() {
        for (subscribedTopic in subscribedTopics) {
            mqttClient.unsubscribe(subscribedTopic)
        }
    }

    fun findAllLatestStatuses(): List<AmrStatus> = amrStatusRepository.findAllLatestStatuses()

    fun findAmrDetail(id: Long): AmrStatus = amrStatusRepository.findAmrStatusById(id)

    fun sendManualControlMessage(serial: String, message: AmrManualControlMessage) =
        mqttPublisher.sendCommand(serial, message)
}