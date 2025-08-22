package com.example.amr_backend.v1.dto

import com.example.amr_backend.v1.entity.Amr
import com.example.amr_backend.v1.entity.AmrStatus
import com.example.amr_backend.v1.entity.State
import io.swagger.v3.oas.annotations.media.Schema

@Schema(description = "MQTT로 수신하는 AMR 상태 메시지")
data class AmrStatusMessage(
    @Schema(description = "AMR 시리얼 번호")
    val serial: String,
    @Schema(description = "AMR 상태")
    val state: State,
    @Schema(description = "x 좌표")
    val x: Double,
    @Schema(description = "y 좌표")
    val y: Double,
    @Schema(description = "속도")
    val speed: Double,
    @Schema(description = "각도")
    val angle: Double,
    @Schema(description = "구역")
    val zone: String? = null
)

fun AmrStatusMessage.toAmrStatus(amr: Amr) = AmrStatus(
    amr = amr,
    state = state,
    x = x,
    y = y,
    speed = speed,
    angle = angle,
    zone = zone
)