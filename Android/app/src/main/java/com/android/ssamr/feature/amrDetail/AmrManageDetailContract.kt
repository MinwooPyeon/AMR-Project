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
    data class ClickWebcam(val ipAddress: String) : AmrDetailIntent()
    data class SelectedWorksheet(val worksheet: String) : AmrDetailIntent()
<<<<<<< HEAD
    data class SelectedChargeStation(val station: String) : AmrDetailIntent()
=======
    data class SelectedChargeStation(val worksheet: String) : AmrDetailIntent()
>>>>>>> origin/develop
}

sealed class AmrDetailEffect {
    data class ShowError(val message: String) : AmrDetailEffect()
<<<<<<< HEAD
    data class ShowMessage(val message: String) : AmrDetailEffect() // toast 메세지용
=======
>>>>>>> origin/develop
    data class NavigateToWebcam(val ipAddress: String) : AmrDetailEffect()
    data object ShowReturnDialog : AmrDetailEffect()
    data object ShowStartDialog : AmrDetailEffect()
}

val worksheetList = listOf("A구역-1", "A구역-2", "A구역-3", "B구역-1", "B구역-2", "B구역-3",
    "C구역-1", "C구역-2", "C구역-3", "D구역-1", "D구역-2", "D구역-3")

val chargeList = listOf("충전소 1", "충전소 2", "충전소 3", "충전소 4")

