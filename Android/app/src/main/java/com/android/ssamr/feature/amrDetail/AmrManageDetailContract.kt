package com.android.ssamr.feature.amrDetail

import androidx.compose.ui.graphics.Color

// core.domain 이동
data class AmrDetailUiModel(
    val name: String,
    val status: String,
    val battery: Int,
    val location: String,
    val speed: String,
    val job: String,
    val model: String,
    val serial: String,
    val firmware: String,
    val lastUpdated: String
)

data class AmrDetailState(
    val amrId: Long? = null,
    val amr: AmrDetailUiModel? = null,
    val showReturnDialog: Boolean = false,
    val showStartDialog: Boolean = false,
    val isLoading: Boolean = false,
    val error: String? = null
)

sealed class AmrDetailIntent {
    data object LoadAmrDetail : AmrDetailIntent()
    data object ClickWebcam : AmrDetailIntent()
    data object ClickManualReturn : AmrDetailIntent()
    data object ClickManualStart : AmrDetailIntent()
}

sealed class AmrDetailEffect {
    data class ShowError(val message: String) : AmrDetailEffect()
    data class ShowSuccess(val message: String) : AmrDetailEffect() // 성공 메세지 관련 추가
    data object NavigateToWebcam : AmrDetailEffect()
    data object ShowReturnDialog : AmrDetailEffect()
    data object ShowStartDialog : AmrDetailEffect()
}

enum class AmrStatus(val display: String, val color: Color) {
    RUNNING("작동중", Color(0xFF23C06C)),    // 초록
    CHARGING("충전중", Color(0xFFFFC700)),   // 노랑
    CHECK("점검중", Color(0xFFFF5050));      // 빨강

    companion object {
        fun fromStatus(status: String): AmrStatus = when (status) {
            "작동중" -> RUNNING
            "충전중" -> CHARGING
            "점검중" -> CHECK
            else -> RUNNING // 기본값
        }
    }
}

