package com.android.ssamr.feature.dashboard.fullscreenmap

import androidx.compose.ui.graphics.ImageBitmap
import com.android.ssamr.feature.dashboard.DashboardAmrStatus

data class AmrMapPosition(
    val id: Long,
    val name: String,
    val x: Float, // px 또는 meter 기준: 추후 확정 필요
    val y: Float,
    val status: DashboardAmrStatus
)

data class FullscreenMapState(
    val isLoading: Boolean = false,
    val amrPositions: List<AmrMapPosition> = emptyList(),
    val mapImage: ImageBitmap? = null, // SLAM이 이미지면
    val error: String? = null,
    val selectedAmrId: Long? = null,
)

sealed interface FullscreenMapIntent {
    data object LoadMap : FullscreenMapIntent
    data class ClickAmr(val id: Long) : FullscreenMapIntent
    data object Close : FullscreenMapIntent
}

sealed interface FullscreenMapEffect {
    data class ShowError(val message: String) : FullscreenMapEffect
    data object NavigateBack : FullscreenMapEffect
    data class ShowAmrDetail(val amrId: Long) : FullscreenMapEffect
}