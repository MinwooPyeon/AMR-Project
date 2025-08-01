package com.android.ssamr.feature.amrWebcam

data class AmrWebcamState(
    val amrName: String = "",
    val lastUpdated: String = "",
    val location: String = "",
    val status: String = "",
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
