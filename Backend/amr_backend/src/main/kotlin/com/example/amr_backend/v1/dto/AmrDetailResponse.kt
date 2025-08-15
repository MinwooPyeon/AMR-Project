package com.example.amr_backend.v1.dto

import com.example.amr_backend.v1.entity.AmrStatus
import com.example.amr_backend.v1.entity.State
import io.swagger.v3.oas.annotations.media.Schema

@Schema(description = "AMR 상세 정보 응답")
data class AmrDetailResponse(
    @Schema(description = "AMR 상태 ID")
    val id: Long,
    @Schema(description = "AMR 이름")
    val name: String,
    @Schema(description = "AMR 상태")
    val state: State,
    @Schema(description = "AMR 속도")
    val speed: Double,
    @Schema(description = "AMR 모델")
    val model: String,
    @Schema(description = "AMR 시리얼 번호")
    val serial: String,
    @Schema(description = "AMR 펌웨어 버전")
    val firmware: String,
    @Schema(description = "AMR IP 주소")
    val ipAddress: String
)

fun AmrStatus.toAmrDetailResponse() = AmrDetailResponse(
    id = id,
    name = amr.name,
    state = state,
    speed = speed,
    model = amr.model,
    serial = amr.serial,
    firmware = amr.firmwareVersion,
    ipAddress = amr.ipAddress
)