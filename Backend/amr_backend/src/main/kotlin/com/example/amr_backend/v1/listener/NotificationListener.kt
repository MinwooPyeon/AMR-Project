package com.example.amr_backend.v1.listener

import com.example.amr_backend.v1.entity.Notification
import com.example.amr_backend.v1.repository.NotificationRepository
import org.springframework.context.event.EventListener
import org.springframework.stereotype.Component
import org.springframework.transaction.annotation.Transactional

@Component
class NotificationListener(
    private val notificationRepository: NotificationRepository,
) {
    @EventListener
    @Transactional
    fun save(notification: Notification) {
        notificationRepository.save(notification)
    }
}