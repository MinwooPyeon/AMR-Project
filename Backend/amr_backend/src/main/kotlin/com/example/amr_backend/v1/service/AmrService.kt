package com.example.amr_backend.v1.service

import com.example.amr_backend.v1.entity.AmrStatus
import com.example.amr_backend.v1.mqtt.MqttClientFactory
import com.example.amr_backend.v1.mqtt.MqttMessageHandler
import com.example.amr_backend.v1.repository.AmrRepository
import com.example.amr_backend.v1.repository.AmrStatusRepository
import com.example.amr_backend.v1.repository.findAmrStatusById
import jakarta.annotation.PostConstruct
import jakarta.annotation.PreDestroy
import org.eclipse.paho.client.mqttv3.MqttClient
import org.springframework.stereotype.Service

typealias Url = String

private const val STATUS_TOPIC = "status"

@Service
class AmrService(
    private val amrRepository: AmrRepository,
    private val amrStatusRepository: AmrStatusRepository,
    private val mqttClientFactory: MqttClientFactory,
    private val mqttMessageHandler: MqttMessageHandler,
) {
    private val clients = mutableMapOf<Url, MqttClient>()

    @PostConstruct
    fun subscribeAllAmrs() {
        val amrs = amrRepository.findAll()

        for (amr in amrs) {
            if (amr.mqttUrl in clients) continue

            val mqttClient = mqttClientFactory.create(amr.mqttUrl).also {
                clients[amr.mqttUrl] = it
            }
            mqttClient.subscribe(STATUS_TOPIC, mqttMessageHandler)
        }
    }

    @PreDestroy
    fun unsubscribeAllAmrs() {
        for ((_, client) in clients) {
            client.unsubscribe(STATUS_TOPIC)
        }
    }

    fun findAllLatestStatuses(): List<AmrStatus> = amrStatusRepository.findAllLatestStatuses()

    fun findAmrDetail(id: Long): AmrStatus = amrStatusRepository.findAmrStatusById(id)
}