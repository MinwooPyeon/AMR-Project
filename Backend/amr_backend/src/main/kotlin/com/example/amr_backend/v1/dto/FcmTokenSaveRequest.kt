package com.example.amr_backend.v1.dto

import io.swagger.v3.oas.annotations.media.Schema

@Schema(description = "FCM 토큰 저장 요청")
data class FcmTokenSaveRequest(
    @Schema(description = "FCM 토큰")
    val token: String
)