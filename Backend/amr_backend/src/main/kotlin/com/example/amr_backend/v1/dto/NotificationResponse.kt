package com.example.amr_backend.v1.dto

import com.example.amr_backend.v1.entity.Notification
import io.swagger.v3.oas.annotations.media.Schema
import java.time.LocalDateTime

@Schema(description = "알림 응답")
data class NotificationResponse(
    @Schema(description = "알림 ID")
    val id: Long,
    @Schema(description = "AMR 시리얼 번호")
    val serial: String,
    @Schema(description = "알림 제목")
    val title: String,
    @Schema(description = "알림 내용")
    val content: String,
    @Schema(description = "알림 발생 구역")
    val area: String,
    @Schema(description = "알림 종류")
    val case: Case,
    @Schema(description = "알림 관련 이미지 URL")
    val image: String?,
    @Schema(description = "읽음 여부")
    val isRead: Boolean,
    @Schema(description = "읽은 시간")
    val readAt: LocalDateTime?,
    @Schema(description = "생성 시간")
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