package com.example.amr_backend.v1.service

import com.example.amr_backend.v1.entity.Notification
import com.example.amr_backend.v1.repository.NotificationRepository
import org.springframework.stereotype.Service

@Service
class NotificationService(
    private val notificationRepository: NotificationRepository,
) {
    fun findAll(): List<Notification> = notificationRepository.findAll()
}