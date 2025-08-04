package com.android.ssamr.feature.amrWebcam

import com.android.ssamr.core.domain.model.AmrDetailStatus

data class AmrWebcamState(
    val amr: AmrDetailStatus? = null,
    val lastUpdated: String = "",
    val rtspUrl: String = "",
    val isFullScreen: Boolean = false,
    val isLoading: Boolean = false,
    val error: String? = null,
)

sealed class AmrWebcamIntent {
    data object LoadAmrInfo : AmrWebcamIntent()
}

sealed class AmrWebcamEffect {
    data class ShowError(val message: String) : AmrWebcamEffect()
}
