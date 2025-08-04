package com.android.ssamr.feature.dashboard

import androidx.compose.ui.graphics.Color

// 화면에 표시될 단일 AMR 정보
data class DashboardAmrUiModel(
    val id: Long,
    val name: String,
    val status: DashboardAmrStatus,
    val location: String,
    val job: String
)

// 화면 전체에 사용될 데이터 목록 및 상태
data class DashboardState(
    val amrList: List<DashboardAmrUiModel> = emptyList(),
    val totalCount: Int = 0,
    val runningCount: Int = 0,
    val chargingCount: Int = 0,
    val checkingCount: Int = 0,
    val isLoading: Boolean = false,
    val error: String? = null
)

// 사용자의 액션 정의
sealed class DashboardIntent {
    data object LoadDashboard : DashboardIntent()
    data class ClickAmrItem(val id: Long) : DashboardIntent()
    data object Refresh : DashboardIntent()

    data object ClickMapExpand : DashboardIntent()
    data object ClickViewAllAmr : DashboardIntent()
}

// UI에서 처리해야 할 일회성 이벤트
sealed class DashboardEffect {
    data class NavigateToAmrDetail(val amrId: Long) : DashboardEffect()

    data object NavigateToMapFullScreen : DashboardEffect()
    data object NavigateToAmrList : DashboardEffect()
}

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
