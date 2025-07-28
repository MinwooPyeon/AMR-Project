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

            try {
                // 더미 데이터
                val amrs = listOf(
                    DashboardAmrUiModel(
                        id = 1L,
                        name = "AMR-001",
                        battery = 85,
                        status = DashboardAmrStatus.RUNNING,
                        location = "A구역",
                        job = "화물 운반"
                    ),
                    DashboardAmrUiModel(
                        id = 2L,
                        name = "AMR-002",
                        battery = 45,
                        status = DashboardAmrStatus.CHARGING,
                        location = "충전소",
                        job = "충전 중"
                    ),
                    DashboardAmrUiModel(
                        id = 3L,
                        name = "AMR-003",
                        battery = 92,
                        status = DashboardAmrStatus.CHECK,
                        location = "B구역",
                        job = "점검 중"
                    ),
                    DashboardAmrUiModel(
                        id = 4L,
                        name = "AMR-004",
                        battery = 67,
                        status = DashboardAmrStatus.RUNNING,
                        location = "C구역",
                        job = "운반"
                    )
                )

                val runningCount = amrs.count { it.status == DashboardAmrStatus.RUNNING }
                val chargingCount = amrs.count { it.status == DashboardAmrStatus.CHARGING }
                val checkingCount = amrs.count { it.status == DashboardAmrStatus.CHECK }

                _state.update {
                    it.copy(
                        amrList = amrs,
                        totalCount = amrs.size,
                        runningCount = runningCount,
                        chargingCount = chargingCount,
                        checkingCount = checkingCount,
                        isLoading = false
                    )
                }

            } catch (e: Exception) {
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
