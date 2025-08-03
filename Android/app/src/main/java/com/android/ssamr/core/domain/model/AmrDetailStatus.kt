package com.android.ssamr.core.domain.model

import androidx.compose.ui.graphics.Color

data class AmrDetailStatus (
    val name: String,
    val status: AmrDetailAction,
    val location: String,
    val speed: String,
    val job: String,
    val model: String,
    val serial: String,
    val firmware: String,
)

enum class AmrDetailAction(val display: String, val color: Color) {
    RUNNING("작동중", Color(0xFF23C06C)),    // 초록
    CHARGING("충전중", Color(0xFFFFC700)),   // 노랑
    CHECKING("점검중", Color(0xFFFF5050));      // 빨강

    companion object {
        fun fromStatus(status: String): AmrDetailAction = when (status) {
            "작동중" -> RUNNING
            "충전중" -> CHARGING
            "점검중" -> CHECKING
            else -> RUNNING // 기본값
        }
    }
}