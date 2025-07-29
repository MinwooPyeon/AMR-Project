package com.android.ssamr.feature.amrDetail

import androidx.lifecycle.SavedStateHandle
import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.android.ssamr.core.usecase.amr.ManualReturnUseCase
import com.android.ssamr.core.usecase.amr.ManualStartUseCase
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
    savedStateHandle: SavedStateHandle,
    private val manualStartUseCase: ManualStartUseCase,
    private val manualReturnUseCase: ManualReturnUseCase,
) : ViewModel() {

    private val _state = MutableStateFlow(AmrDetailState())
    val state: StateFlow<AmrDetailState> = _state.onSubscription {
        sendIntent(AmrDetailIntent.LoadAmrDetail)
    }.stateIn(
        scope = viewModelScope,
        started = SharingStarted.WhileSubscribed(5000),
        initialValue = AmrDetailState()
    )

    val amrId = requireNotNull(savedStateHandle.get<Long>("amrId")) {
        "amrId is null in AmrDetailViewModel"
    }

    private val _effect = MutableSharedFlow<AmrDetailEffect>()
    val effect: SharedFlow<AmrDetailEffect> = _effect

    fun sendIntent(intent: AmrDetailIntent) {
        when (intent) {
            is AmrDetailIntent.LoadAmrDetail -> {
                viewModelScope.launch {
                    _state.value = _state.value.copy(isLoading = true)
                    delay(500) // 실제 API 호출로 교체
                    _state.value = _state.value.copy(
                        isLoading = false,
                        amr = sampleAmrDetail
                    )
                }
            }

            is AmrDetailIntent.ClickWebcam -> {
                viewModelScope.launch {
                    _effect.emit(AmrDetailEffect.NavigateToWebcam)
                }
            }

            is AmrDetailIntent.ClickManualStart -> {
                viewModelScope.launch {
                    _state.value = _state.value.copy(showStartDialog = true)

                    val result = manualStartUseCase(amrId)
                    delay(1000)

                    _state.value = _state.value.copy(showStartDialog = false)

                    if (result.isSuccess) {
                        emitEffect(AmrDetailEffect.ShowSuccess("출발 요청이 전송되었습니다."))
                    } else {
                        emitEffect(
                            AmrDetailEffect.ShowError(
                                "출발 요청 실패: ${result.exceptionOrNull()?.message.orEmpty()}"
                            )
                        )
                    }
                }
            }

            is AmrDetailIntent.ClickManualReturn -> {
                viewModelScope.launch {
                    _state.value = _state.value.copy(showReturnDialog = true)

                    val result = manualReturnUseCase(amrId)
                    delay(1000)

                    _state.value = _state.value.copy(showReturnDialog = false)

                    if (result.isSuccess) {
                        emitEffect(AmrDetailEffect.ShowSuccess("복귀 요청이 전송되었습니다."))
                    } else {
                        emitEffect(
                            AmrDetailEffect.ShowError(
                                "복귀 요청 실패: ${result.exceptionOrNull()?.message.orEmpty()}"
                            )
                        )
                    }
                }
            }
        }
    }

    // 헬퍼 함수는 sendIntent 외부, 클래스 내부에 위치
    private fun emitEffect(effect: AmrDetailEffect) {
        viewModelScope.launch {
            _effect.emit(effect)
        }
    }
}
