package com.android.ssamr.feature.dashboard

import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.android.ssamr.core.domain.model.DashboardAmr
import com.android.ssamr.core.domain.model.DashboardAmrStatus
import com.android.ssamr.core.domain.usecase.amr.GetDashboardAmrsUseCase
import dagger.hilt.android.lifecycle.HiltViewModel
import kotlinx.coroutines.flow.*
import kotlinx.coroutines.launch
import javax.inject.Inject

@HiltViewModel
class DashboardViewModel @Inject constructor(
    private val getDashboardAmrsUseCase: GetDashboardAmrsUseCase
) : ViewModel() {

    private val USE_DUMMY_DATA = false // true: 더미 사용 / false: 실제 API 사용

    private val _state = MutableStateFlow(DashboardState())
    val state: StateFlow<DashboardState> = _state.asStateFlow()

    private val _effect = MutableSharedFlow<DashboardEffect>()
    val effect: SharedFlow<DashboardEffect> = _effect.asSharedFlow()

    init {
        onIntent(DashboardIntent.LoadDashboard)
    }

    fun onIntent(intent: DashboardIntent) {
        when (intent) {
            is DashboardIntent.LoadDashboard,
            is DashboardIntent.Refresh -> {
                loadDashboard()
            }

            is DashboardIntent.ClickAmrItem -> {
                emitEffect(DashboardEffect.NavigateToAmrDetail(intent.id))
            }

            is DashboardIntent.ClickMapExpand -> {
                emitEffect(DashboardEffect.NavigateToMapFullScreen)
            }

            is DashboardIntent.ClickViewAllAmr -> {
                emitEffect(DashboardEffect.NavigateToAmrList)
            }
        }
    }

    private fun loadDashboard() {
        viewModelScope.launch {
            _state.update { it.copy(isLoading = true, error = null) }

            runCatching {
                val amrs: List<DashboardAmr> = if (USE_DUMMY_DATA) {
                    listOf(
                        DashboardAmr(1L, "AMR-001", DashboardAmrStatus.RUNNING, "A구역"),
                        DashboardAmr(2L, "AMR-002", DashboardAmrStatus.CHARGING, "충전소"),
                        DashboardAmr(3L, "AMR-003", DashboardAmrStatus.CHECK, "B구역"),
                        DashboardAmr(4L, "AMR-004", DashboardAmrStatus.RUNNING, "C구역")
                    )
                } else {
                    getDashboardAmrsUseCase().getOrThrow()
                }

                val runningCount = amrs.count { it.status == DashboardAmrStatus.RUNNING }
                val chargingCount = amrs.count { it.status == DashboardAmrStatus.CHARGING }
                val checkingCount = amrs.count { it.status == DashboardAmrStatus.CHECK }

                DashboardState(
                    amrList = amrs,
                    totalCount = amrs.size,
                    runningCount = runningCount,
                    chargingCount = chargingCount,
                    checkingCount = checkingCount,
                    isLoading = false
                )
            }.onSuccess { newState ->
                _state.update { newState }
            }.onFailure {
                _state.update { it.copy(isLoading = false, error = "데이터 로딩 실패") }
            }
        }
    }

    private fun emitEffect(effect: DashboardEffect) {
        viewModelScope.launch {
            _effect.emit(effect)
        }
    }
}
