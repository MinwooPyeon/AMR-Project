package com.android.ssamr.feature.amrDetail

import androidx.compose.ui.graphics.Color
import com.android.ssamr.core.domain.model.AmrDetailStatus

data class AmrDetailState(
    val amrId: Long? = null,
    val amr: AmrDetailStatus? = null,
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
    data object NavigateToWebcam : AmrDetailEffect()
    data object ShowReturnDialog : AmrDetailEffect()
    data object ShowStartDialog : AmrDetailEffect()
}
