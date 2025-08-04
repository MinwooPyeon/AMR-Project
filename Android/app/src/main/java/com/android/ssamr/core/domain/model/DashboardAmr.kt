package com.android.ssamr.core.domain.model

import androidx.compose.ui.graphics.Color

data class DashboardAmr(
    val id: Long,
    val name: String,
    val status: DashboardAmrStatus,
    val location: String,
    val job: String
)

// 상태 정보 Enum
enum class DashboardAmrStatus(val display: String, val color: Color) {
    RUNNING("작동중", Color(0xFF23C06C)),     // 초록
    CHARGING("충전중", Color(0xFFFFC700)),    // 노랑
    CHECK("점검중", Color(0xFFFF5050));       // 빨강

    companion object {
        fun from(status: String): DashboardAmrStatus = when (status) {
            "작동중" -> RUNNING
            "충전중" -> CHARGING
            "점검중" -> CHECK
            else -> CHECK
        }
    }
}