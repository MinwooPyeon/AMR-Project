package com.example.amr_backend.v1.service

import com.example.amr_backend.v1.dto.TopicMessage
import com.example.amr_backend.v1.entity.AmrStatus
import com.example.amr_backend.v1.exception.NoSuchAmrStatus
import com.example.amr_backend.v1.repository.AmrStatusRepository
import org.springframework.stereotype.Service

@Service
class AmrService(
    private val amrStatusRepository: AmrStatusRepository,
    private val topicPublisher: TopicPublisher,
) {
    fun findAllLatestStatuses(): List<AmrStatus> = amrStatusRepository.findAllLatestStatuses()

    fun findLatestStatusBySerial(serial: String): AmrStatus =
        amrStatusRepository.findLatestStatusBySerial(serial).orElseThrow {
            NoSuchAmrStatus("시리얼이 ${serial}인 AMR의 상태가 없습니다.")
        }

    fun sendManualControlMessage(serial: String, message: TopicMessage.AmrManualControlMessage) =
        topicPublisher.publish("control", message)
}