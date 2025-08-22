package com.example.amr_backend.v1.dto

import io.swagger.v3.oas.annotations.media.Schema

@Schema(description = "알림 읽음 처리 요청")
data class NotificationReadUpdateRequest(
    @Schema(description = "읽음 여부")
    val isRead: Boolean
)
