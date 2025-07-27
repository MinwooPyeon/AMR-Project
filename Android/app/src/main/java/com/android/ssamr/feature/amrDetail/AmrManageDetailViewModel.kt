package com.android.ssamr.feature.amrDetail

import androidx.lifecycle.SavedStateHandle
import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
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
class AmrDetailViewModel @Inject constructor(
    // private val getAmrDetailUseCase: GetAmrDetailUseCase
    savedStateHandle: SavedStateHandle
) : ViewModel() {

    private val _state = MutableStateFlow(AmrDetailState())
    val state: StateFlow<AmrDetailState> = _state.onSubscription {
        sendIntent(AmrDetailIntent.LoadAmrDetail)
    }.stateIn(
        scope = viewModelScope,
        started = SharingStarted.WhileSubscribed(stopTimeoutMillis = 5000),
        initialValue = AmrDetailState()
    )

    val amrId = requireNotNull(savedStateHandle.get<Long>("amrId")) {
        "amd id is null in AmrDetailViewModel"
    }

    private val _effect = MutableSharedFlow<AmrDetailEffect>()
    val effect: SharedFlow<AmrDetailEffect> = _effect

    fun sendIntent(intent: AmrDetailIntent) {
        when (intent) {
            is AmrDetailIntent.LoadAmrDetail -> {
                // 실제로는 서버 API 호출 (예시로 딜레이 후 샘플 데이터)
                viewModelScope.launch {
                    _state.value = _state.value.copy(isLoading = true)
//                    val amr = getAmrDetailUseCase(amrId)
                    delay(500)
                    _state.value = _state.value.copy(
                        isLoading = false,
                        amr = sampleAmrDetail
                    )
                }
            }

            is AmrDetailIntent.ClickWebcam -> {
                viewModelScope.launch { _effect.emit(AmrDetailEffect.NavigateToWebcam) }
            }

            is AmrDetailIntent.ClickManualReturn -> {
                _state.value = _state.value.copy(showReturnDialog = true)
                viewModelScope.launch {
                    delay(2000)
                    _state.value = _state.value.copy(showReturnDialog = false)
                }
            }

            is AmrDetailIntent.ClickManualStart -> {
                _state.value = _state.value.copy(showStartDialog = true)
                viewModelScope.launch {
                    delay(2000)
                    _state.value = _state.value.copy(showStartDialog = false)
                }
            }
        }
    }
}