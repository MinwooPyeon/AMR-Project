package com.android.ssamr.feature.dashboard

import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.android.ssamr.core.domain.model.AmrAction
import com.android.ssamr.core.domain.model.AmrStatus
import com.android.ssamr.core.domain.model.DashboardAmr
import com.android.ssamr.core.domain.model.DashboardAmrStatus
import com.android.ssamr.core.domain.repository.AmrRepository
import dagger.hilt.android.lifecycle.HiltViewModel
import kotlinx.coroutines.flow.*
import kotlinx.coroutines.launch
import javax.inject.Inject

@HiltViewModel
class DashboardViewModel @Inject constructor(
    private val amrRepository: AmrRepository
) : ViewModel() {

    private val USE_DUMMY_DATA = false

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
            is DashboardIntent.Refresh -> loadDashboard()
            is DashboardIntent.ClickAmrItem -> emitEffect(DashboardEffect.NavigateToAmrDetail(intent.id))
            is DashboardIntent.ClickMapExpand -> emitEffect(DashboardEffect.NavigateToMapFullScreen)
            is DashboardIntent.ClickViewAllAmr -> emitEffect(DashboardEffect.NavigateToAmrList)
        }
    }

    private fun loadDashboard() {
        viewModelScope.launch {
            _state.update { it.copy(isLoading = true, error = null) }

            runCatching {
                val amrStatuses: List<AmrStatus> = if (USE_DUMMY_DATA) {
                    listOf(
                        AmrStatus(1L, "AMR-001", AmrAction.RUNNING, 1.0, 2.0, "1.2m/s", "A라인"),
                        AmrStatus(2L, "AMR-002", AmrAction.CHARGING, 3.0, 4.0, "0.0m/s", "충전소"),
                        AmrStatus(3L, "AMR-003", AmrAction.CHECKING, 5.0, 6.0, "0.0m/s", "B라인")
                    )
                } else {
                    amrRepository.getAmrList()
                }

                val amrs: List<DashboardAmr> = amrStatuses.map { it.toDashboardAmr() }

                val runningCount = amrs.count { it.status == DashboardAmrStatus.RUNNING }
                val chargingCount = amrs.count { it.status == DashboardAmrStatus.CHARGING }
                val checkingCount = amrs.count { it.status == DashboardAmrStatus.CHECKING }

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

    fun AmrStatus.toDashboardAmr(): DashboardAmr {
        return DashboardAmr(
            id = this.id,
            name = this.name,
            status = when (this.status) {
                AmrAction.RUNNING -> DashboardAmrStatus.RUNNING
                AmrAction.CHARGING -> DashboardAmrStatus.CHARGING
                AmrAction.CHECKING -> DashboardAmrStatus.CHECKING
            },
            location = this.job // 필요시 locationX/Y 기반으로 별도 생성 가능
        )
    }

    private fun emitEffect(effect: DashboardEffect) {
        viewModelScope.launch {
            _effect.emit(effect)
        }
    }
}
