package com.android.ssamr.feature.amr

import android.util.Log
import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.android.ssamr.core.domain.model.AmrCategory
import com.android.ssamr.core.domain.model.AmrAction
import com.android.ssamr.core.domain.model.AmrStatus
import com.android.ssamr.core.domain.usecase.amr.GetAmrListUseCase
import dagger.hilt.android.lifecycle.HiltViewModel
import kotlinx.coroutines.CancellationException
import kotlinx.coroutines.Job
import kotlinx.coroutines.delay
import kotlinx.coroutines.flow.MutableSharedFlow
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.SharedFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asSharedFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.isActive
import kotlinx.coroutines.launch
import javax.inject.Inject

@HiltViewModel
class AmrManageViewModel @Inject constructor(
    private val getAmrListUseCase: GetAmrListUseCase
) : ViewModel() {

    private val _state = MutableStateFlow(AmrState())
    val state: StateFlow<AmrState> = _state.asStateFlow()

    private val _effect = MutableSharedFlow<AmrEffect>()
    val effect: SharedFlow<AmrEffect>
        get() = _effect.asSharedFlow()

    private var pollingJob: Job? = null

    init {
        fetchAmrList()
    }

    fun sendIntent(intent: AmrIntent) {
        when (intent) {
            is AmrIntent.ClickAmrCategory -> {
                val filtered = filterList(_state.value.fullAmrList, intent.category)
                _state.value = _state.value.copy(
                    selectedCategory = intent.category,
                    amrList = filtered
                )
            }

            is AmrIntent.ClickAmrManageCard -> {
                viewModelScope.launch { _effect.emit(AmrEffect.NavigateToAmrDetail(intent.serial)) }
            }
        }
    }

    private fun fetchAmrList(intervalMs: Long = 30000L) {
        pollingJob?.cancel()
        pollingJob = viewModelScope.launch {
            while (isActive) {
                try {
                    val list = getAmrListUseCase()
                    val filitered = filterList(list, _state.value.selectedCategory)
                    val counts = AmrCategory.values().associateWith { cat ->
                        when (cat) {
                            AmrCategory.ALL -> list.size
                            AmrCategory.RUNNING -> list.count { it.state == AmrAction.RUNNING }
                            AmrCategory.CHARGING -> list.count { it.state == AmrAction.CHARGING }
                            AmrCategory.CHECKING -> list.count { it.state == AmrAction.CHECKING }
                        }
                    }
                    _state.value = _state.value.copy(
                        fullAmrList = list,
                        amrList = filitered,
                        categoryCounts = counts,
                        isLoading = false,
                        error = null
                    )
                    Log.d("AMR", "fetchAmrList: ${list}")
                } catch (e: Exception) {
                    if (e is CancellationException) throw e
                    e.printStackTrace()
                    _effect.emit(AmrEffect.ShowError("Amr 리스트 로드 실패: ${e.message}"))
                }
                delay(intervalMs)
            }
        }
    }

    fun refreshAmrList() {
        viewModelScope.launch {
            Log.d("AmrManage", "refreshAmrList: ")
            try {
                val list = getAmrListUseCase()
                val filtered = filterList(list, _state.value.selectedCategory)
                val counts = AmrCategory.values().associateWith { cat ->
                    when (cat) {
                        AmrCategory.ALL -> list.size
                        AmrCategory.RUNNING -> list.count { it.state == AmrAction.RUNNING }
                        AmrCategory.CHARGING -> list.count { it.state == AmrAction.CHARGING }
                        AmrCategory.CHECKING -> list.count { it.state == AmrAction.CHECKING }
                    }
                }
                _state.value = _state.value.copy(
                    fullAmrList = list,
                    amrList = filtered,
                    categoryCounts = counts,
                    isLoading = false,
                    error = null
                )
                Log.d("AmrManage", "refreshAmrList: 111")
            } catch (e: Exception) {
                _effect.emit(AmrEffect.ShowError("AMR 리스트 로드 실패: ${e.message}"))
            }
        }
    }

    private fun filterList(list: List<AmrStatus>, category: AmrCategory): List<AmrStatus> {
        return when (category) {
            AmrCategory.ALL -> list
            AmrCategory.RUNNING -> list.filter { it.state == AmrAction.RUNNING }
            AmrCategory.CHARGING -> list.filter { it.state == AmrAction.CHARGING }
            AmrCategory.CHECKING -> list.filter { it.state == AmrAction.CHECKING }
        }

    }
}
