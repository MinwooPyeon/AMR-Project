package com.example.amr_backend.v1.service

import com.example.amr_backend.v1.dto.TopicMessage
import com.example.amr_backend.v1.entity.AmrStatus
import com.example.amr_backend.v1.repository.AmrStatusRepository
import org.springframework.stereotype.Service

@Service
class AmrService(
    private val amrStatusRepository: AmrStatusRepository,
    private val topicPublisher: TopicPublisher,
) {
    fun findAllLatestStatuses(): List<AmrStatus> = amrStatusRepository.findAllLatestStatuses()

    fun findLatestStatusBySerial(serial: String): AmrStatus = amrStatusRepository.findLatestStatusBySerial(serial)

    fun sendManualControlMessage(serial: String, message: TopicMessage.AmrManualControlMessage) =
        topicPublisher.publish("control", message)
}