package com.example.amr_backend.v1.dto

import io.swagger.v3.oas.annotations.media.Schema

sealed interface TopicMessage {
    @Schema(description = "AMR 수동 제어 메시지")
    data class AmrManualControlMessage(
        @Schema(description = "AMR 시리얼 번호")
        val serial: String,
        @Schema(description = "목적지")
        val area: String
    ) : TopicMessage
}