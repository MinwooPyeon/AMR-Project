package com.android.ssamr.feature.amrDetail

import androidx.lifecycle.SavedStateHandle
import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.android.ssamr.core.domain.usecase.amr.GetAmrDetailUseCase
import com.android.ssamr.core.domain.usecase.amr.ManualControlUseCase
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
    private val getAmrDetailUseCase: GetAmrDetailUseCase,
    private val manualControlUseCase: ManualControlUseCase,
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

    val serial = requireNotNull(savedStateHandle.get<String>("serial")) {
        "serial is null in AmrDetailViewModel"
    }

    private val _effect = MutableSharedFlow<AmrDetailEffect>()
    val effect: SharedFlow<AmrDetailEffect> = _effect

    fun sendIntent(intent: AmrDetailIntent) {
        when (intent) {
            is AmrDetailIntent.LoadAmrDetail -> {
                viewModelScope.launch {
                    _state.value = _state.value.copy(isLoading = true)
                    try {
                        val amr = getAmrDetailUseCase(serial)
                        delay(500)
                        _state.value = _state.value.copy(
                            isLoading = false,
                            amr = amr,
                        )
                    } catch (e: Exception) {
                        _state.value = _state.value.copy(
                            isLoading = false,
                            error = e.message ?: "상세정보 로드 실패"
                        )
                        _effect.emit(AmrDetailEffect.ShowError(e.message ?: "상세정보 로드 실패"))
                    }
                }
            }

            is AmrDetailIntent.ClickWebcam -> {
                viewModelScope.launch {
                    _effect.emit(AmrDetailEffect.NavigateToWebcam(intent.ipAddress))
                }
            }

            is AmrDetailIntent.SelectedWorksheet -> {
                _state.value = _state.value.copy(showStartDialog = true)
                viewModelScope.launch {
                    val result = manualControlUseCase(serial, intent.worksheet)
                    if (result.isFailure) {
                        _effect.emit(AmrDetailEffect.ShowError("작업지 이동 실패"))
                    } else {
                        _effect.emit(AmrDetailEffect.ShowMessage("${intent.worksheet}로 이동 요청을 보냈습니다."))
                    }
                    delay(2000)
                    _state.value = _state.value.copy(showStartDialog = false)
                }
            }

            is AmrDetailIntent.SelectedChargeStation -> {
                _state.value = _state.value.copy(showReturnDialog = true)
                viewModelScope.launch {
                    val result = manualControlUseCase(serial, intent.station)
                    if (result.isFailure) {
                        _effect.emit(AmrDetailEffect.ShowError("충전소 이동 실패"))
                    } else {
                        _effect.emit(AmrDetailEffect.ShowMessage("${intent.station}로 이동 요청을 보냈습니다."))
                    }
                    delay(2000)
                    _state.value = _state.value.copy(showReturnDialog = false)
                }
            }
        }
    }
}
