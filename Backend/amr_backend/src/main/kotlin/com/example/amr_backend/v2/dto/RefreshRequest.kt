package com.example.amr_backend.v2.dto

import io.swagger.v3.oas.annotations.media.Schema

@Schema(description = "토큰 재발급 요청")
data class RefreshRequest(
    @Schema(description = "리프레시 토큰")
    val refreshToken: String
)
