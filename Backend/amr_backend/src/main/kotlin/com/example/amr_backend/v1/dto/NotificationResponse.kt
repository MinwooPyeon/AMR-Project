package com.example.amr_backend.v1.dto

import com.example.amr_backend.v1.entity.Notification
import java.time.LocalDateTime

data class NotificationResponse(
    val id: Long,
    val serial: String,
    val title: String,
    val content: String,
    val area: String,
    val case: Case,
    val image: String?,
    val isRead: Boolean,
    val readAt: LocalDateTime?,
    val createAt: LocalDateTime,
)

fun Notification.toNotificationResponse() = NotificationResponse(
    id = id,
    serial = serial,
    title = title,
    content = content,
    area = area,
    case = case,
    image = image,
    isRead = isRead,
    readAt = readAt,
    createAt = createAt
)