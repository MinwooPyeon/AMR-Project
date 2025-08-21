package com.example.amr_backend.v1.dto

import com.example.amr_backend.v1.entity.AmrStatus
import com.example.amr_backend.v1.entity.State
import io.swagger.v3.oas.annotations.media.Schema

@Schema(description = "AMR 상태 정보 응답")
data class AmrStatusResponse(
    @Schema(description = "AMR 상태 ID")
    val id: Long,
    @Schema(description = "AMR 시리얼 번호")
    val serial: String,
    @Schema(description = "AMR 이름")
    val name: String,
    @Schema(description = "AMR 상태")
    val state: State,
    @Schema(description = "AMR 속도")
    val speed: Double,
    @Schema(description = "x 좌표")
    val x: Double,
    @Schema(description = "y 좌표")
    val y: Double,
    @Schema(description = "구역")
    val zone: String?,
)

fun AmrStatus.toAmrStatusResponse() = AmrStatusResponse(
    id = id,
    serial = amr.serial,
    name = amr.name,
    state = state,
    speed = speed,
    x = x,
    y = y,
    zone = zone
)