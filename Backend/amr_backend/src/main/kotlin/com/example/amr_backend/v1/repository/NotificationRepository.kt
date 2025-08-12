package com.example.amr_backend.v1.repository

import com.example.amr_backend.v1.entity.Notification
import com.example.amr_backend.v1.exception.NoSuchNotification
import org.springframework.data.jpa.repository.JpaRepository

interface NotificationRepository : JpaRepository<Notification, Long> {
}

fun NotificationRepository.getNotificationById(id: Long): Notification = findById(id).orElseThrow {
    NoSuchNotification("id가 ${id}인 알림이 없습니다.")
}