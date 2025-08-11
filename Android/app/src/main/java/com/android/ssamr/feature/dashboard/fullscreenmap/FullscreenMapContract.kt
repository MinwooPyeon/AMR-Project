package com.android.ssamr.feature.dashboard.fullscreenmap

import androidx.compose.ui.graphics.ImageBitmap
import com.android.ssamr.core.domain.model.AmrMapPosition

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
    data object ClearSelectedAmr : FullscreenMapIntent //selectedAmrId 선택 해제를 위한
}