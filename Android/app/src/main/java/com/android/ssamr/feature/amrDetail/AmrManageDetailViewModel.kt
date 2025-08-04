package com.android.ssamr.feature.amrDetail

import androidx.lifecycle.SavedStateHandle
import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.android.ssamr.core.domain.usecase.amr.GetAmrDetailUseCase
import com.android.ssamr.core.domain.usecase.amr.ManualReturnUseCase
import com.android.ssamr.core.domain.usecase.amr.ManualStartUseCase
import dagger.hilt.android.lifecycle.HiltViewModel
import kotlinx.coroutines.delay
import kotlinx.coroutines.flow.*
import kotlinx.coroutines.launch
import javax.inject.Inject

@HiltViewModel
class AmrDetailViewModel @Inject constructor(
    private val getAmrDetailUseCase: GetAmrDetailUseCase,
    private val manualReturnUseCase: ManualReturnUseCase,
    private val manualStartUseCase: ManualStartUseCase,
    savedStateHandle: SavedStateHandle
) : ViewModel() {

    private val _state = MutableStateFlow(AmrDetailState())
    val state: StateFlow<AmrDetailState> = _state

    private val _effect = MutableSharedFlow<AmrDetailEffect>()
    val effect: SharedFlow<AmrDetailEffect> = _effect

    private val amrId: Long = requireNotNull(savedStateHandle["amrId"]) {
        "amrId is null in AmrDetailViewModel"
    }

    init {
        loadAmrDetail()
    }

    fun sendIntent(intent: AmrDetailIntent) {
        when (intent) {
            is AmrDetailIntent.LoadAmrDetail -> loadAmrDetail()
            is AmrDetailIntent.ClickWebcam -> navigateToWebcam(intent.ipAddress)
            is AmrDetailIntent.ClickManualReturn -> requestManualReturn()
            is AmrDetailIntent.ClickManualStart -> requestManualStart()
        }
    }

    private fun loadAmrDetail() {
        viewModelScope.launch {
            _state.update { it.copy(isLoading = true) }
            try {
                val amr = getAmrDetailUseCase(amrId)
                delay(500)
                _state.update { it.copy(isLoading = false, amr = amr) }
            } catch (e: Exception) {
                val message = e.message ?: "상세정보 로드 실패"
                _state.update { it.copy(isLoading = false, error = message) }
                _effect.emit(AmrDetailEffect.ShowError(message))
            }
        }
    }

    private fun requestManualReturn() {
        _state.update { it.copy(showReturnDialog = true) }
        viewModelScope.launch {
            val result = manualReturnUseCase(amrId)
            if (result.isFailure) {
                _effect.emit(AmrDetailEffect.ShowError("복귀 요청 실패"))
            }
            delay(2000)
            _state.update { it.copy(showReturnDialog = false) }
        }
    }

    private fun requestManualStart() {
        _state.update { it.copy(showStartDialog = true) }
        viewModelScope.launch {
            val result = manualStartUseCase(amrId)
            if (result.isFailure) {
                _effect.emit(AmrDetailEffect.ShowError("출발 요청 실패"))
            }
            delay(2000)
            _state.update { it.copy(showStartDialog = false) }
        }
    }

    private fun navigateToWebcam(ipAddress: String) {
        viewModelScope.launch {
            _effect.emit(AmrDetailEffect.NavigateToWebcam(ipAddress))
        }
    }
}
