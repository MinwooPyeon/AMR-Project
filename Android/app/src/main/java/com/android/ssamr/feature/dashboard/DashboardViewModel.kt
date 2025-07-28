package com.android.ssamr.feature.dashboard

import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import dagger.hilt.android.lifecycle.HiltViewModel
import kotlinx.coroutines.flow.*
import kotlinx.coroutines.launch
import javax.inject.Inject

@HiltViewModel
class DashboardViewModel @Inject constructor(
    // TODO: 실제 데이터 로딩용 UseCase 또는 Repository 주입 예정
) : ViewModel() {

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
                val amrs = listOf(
                    DashboardAmrUiModel(1L, "AMR-001", 85, DashboardAmrStatus.RUNNING, "A구역", "화물 운반"),
                    DashboardAmrUiModel(2L, "AMR-002", 45, DashboardAmrStatus.CHARGING, "충전소", "충전 중"),
                    DashboardAmrUiModel(3L, "AMR-003", 92, DashboardAmrStatus.CHECK, "B구역", "점검 중"),
                    DashboardAmrUiModel(4L, "AMR-004", 67, DashboardAmrStatus.RUNNING, "C구역", "운반")
                )

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
            }.onFailure { throwable ->
                _state.update { it.copy(isLoading = false, error = "데이터를 불러오는 데 실패했습니다.") }
                emitEffect(DashboardEffect.ShowError("네트워크 오류가 발생했습니다."))
            }
        }
    }


    private fun emitEffect(effect: DashboardEffect) {
        viewModelScope.launch {
            _effect.emit(effect)
        }
    }
}
