package com.android.ssamr.feature.amrWebcam

import androidx.lifecycle.SavedStateHandle
import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import dagger.hilt.android.lifecycle.HiltViewModel
import kotlinx.coroutines.flow.MutableSharedFlow
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.SharedFlow
import kotlinx.coroutines.flow.SharingStarted
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.onSubscription
import kotlinx.coroutines.flow.stateIn
import javax.inject.Inject

@HiltViewModel
class AmrWebcamViewModel @Inject constructor(
//    private val getAmrInfoUsecase: GetAmrInfoUsecase
    savedStateHandle: SavedStateHandle
) : ViewModel() {
    private val amrId: Long = requireNotNull(savedStateHandle["amrId"]) { "amrId가 없습니다. "}
    private val ipAddress: String = requireNotNull(savedStateHandle["ipAddress"]) { "ip주소가 없습니다. " }

    private fun makeRtspUrl(ip: String, port: Int = 554, path: String = "live.sdp") =
        "rtsp://$ip:$port/$path"

    private val _state = MutableStateFlow(
        AmrWebcamState(
            amrName = "AMR-$amrId",
            rtspUrl = makeRtspUrl(ipAddress)
        )
    )

    val state: StateFlow<AmrWebcamState> = _state.onSubscription {
        sendIntent(AmrWebcamIntent.LoadAmrInfo)
    }.stateIn(
        scope = viewModelScope,
        started = SharingStarted.WhileSubscribed(stopTimeoutMillis = 5000),
        initialValue = AmrWebcamState()
    )

    private val _effect = MutableSharedFlow<AmrWebcamEffect>()
    val effect: SharedFlow<AmrWebcamEffect> = _effect

    fun sendIntent(intent: AmrWebcamIntent) {
        when(intent) {
            is AmrWebcamIntent.LoadAmrInfo -> {

            }
        }
    }
}
