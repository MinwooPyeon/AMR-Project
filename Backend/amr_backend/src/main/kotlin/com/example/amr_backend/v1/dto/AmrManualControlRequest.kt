package com.example.amr_backend.v1.dto

import io.swagger.v3.oas.annotations.media.Schema

@Schema(description = "AMR 수동 제어 요청")
data class AmrManualControlRequest(
    @Schema(description = "AMR 시리얼 번호")
    val serial: String,
    @Schema(description = "목적지")
    val area: String,
)

fun AmrManualControlRequest.toAmrManualControlMessage() = TopicMessage.AmrManualControlMessage(
    serial = serial,
    area = area
)