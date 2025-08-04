package com.android.ssamr.feature.dashboard

import androidx.compose.ui.graphics.Color
import com.android.ssamr.core.domain.model.DashboardAmr

// 화면 전체에 사용될 데이터 목록 및 상태
data class DashboardState(
    val amrList: List<DashboardAmr> = emptyList(),
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
