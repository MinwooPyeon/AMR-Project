package com.example.amr_backend.v1.service

import com.example.amr_backend.v1.entity.Notification
import com.example.amr_backend.v1.repository.NotificationRepository
import com.example.amr_backend.v1.repository.getNotificationById
import jakarta.persistence.EntityNotFoundException
import org.slf4j.LoggerFactory
import org.springframework.stereotype.Service

@Service
class NotificationService(
    private val notificationRepository: NotificationRepository,
) {
    private val logger = LoggerFactory.getLogger(this::class.java)

    fun findAll(): List<Notification> = notificationRepository.findAll()

    fun getById(id: Long): Notification = try {
        notificationRepository.getNotificationById(id)
    } catch (e: EntityNotFoundException) {
        logger.error("no such notification whose id is {}", id)
        logger.error(e.stackTraceToString())
        throw e
    }
}