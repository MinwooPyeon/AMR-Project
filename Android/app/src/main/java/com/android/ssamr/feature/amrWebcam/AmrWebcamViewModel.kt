package com.android.ssamr.feature.amrWebcam

import androidx.lifecycle.SavedStateHandle
import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.android.ssamr.core.domain.model.AmrDetailStatus
import com.android.ssamr.core.domain.usecase.amr.GetAmrDetailUseCase
import dagger.hilt.android.lifecycle.HiltViewModel
import kotlinx.coroutines.delay
import kotlinx.coroutines.flow.MutableSharedFlow
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.SharedFlow
import kotlinx.coroutines.flow.SharingStarted
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.onSubscription
import kotlinx.coroutines.flow.stateIn
import kotlinx.coroutines.launch
import javax.inject.Inject

@HiltViewModel
class AmrWebcamViewModel @Inject constructor(
    private val getAmrDetailUseCase: GetAmrDetailUseCase,
    savedStateHandle: SavedStateHandle
) : ViewModel() {
    private val amrId: Long = requireNotNull(savedStateHandle["amrId"]) { "amrId가 없습니다. "}
    private val ipAddress: String = requireNotNull(savedStateHandle["ipAddress"]) { "ip주소가 없습니다. " }

    private val testRtspUrl = "192.168.100.217"

    private fun makeRtspUrl(ip: String, port: Int = 8554, path: String = "mystream") =
        "rtsp://$ip:$port/$path"

    private val _state = MutableStateFlow(
        AmrWebcamState(
//            rtspUrl = makeRtspUrl(ipAddress)
            rtspUrl = makeRtspUrl(testRtspUrl),
            lastUpdated = "2025-08-01 12:34:56",
        )
    )

    val state: StateFlow<AmrWebcamState> = _state.onSubscription {
        sendIntent(AmrWebcamIntent.LoadAmrInfo)
    }.stateIn(
        scope = viewModelScope,
        started = SharingStarted.WhileSubscribed(stopTimeoutMillis = 5000),
        initialValue = AmrWebcamState(
            rtspUrl = makeRtspUrl(testRtspUrl),
            lastUpdated = "2025-08-01 12:34:56",
        )
    )

    private val _effect = MutableSharedFlow<AmrWebcamEffect>()
    val effect: SharedFlow<AmrWebcamEffect> = _effect

    fun sendIntent(intent: AmrWebcamIntent) {
        when(intent) {
            is AmrWebcamIntent.LoadAmrInfo -> {
                viewModelScope.launch {
                    _state.value = _state.value.copy(isLoading = true)
                    try {
                        val amr = getAmrDetailUseCase(amrId)
                        delay(500)
                        _state.value = _state.value.copy(
                            isLoading = false,
                            amr = amr
                        )
                    } catch (e: Exception) {
                        _state.value = _state.value.copy(
                            isLoading = false,
                            error = e.message?: "실시간 정보 로드 실패"
                        )
                        _effect.emit(AmrWebcamEffect.ShowError(e.message ?: "실사간 정보 로드 실패"))
                    }
                }
            }
        }
    }
}
