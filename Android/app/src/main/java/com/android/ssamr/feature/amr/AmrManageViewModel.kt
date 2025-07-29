package com.android.ssamr.feature.amr

import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.android.ssamr.core.domain.usecase.amr.GetAmrListUseCase
import dagger.hilt.android.lifecycle.HiltViewModel
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
                viewModelScope.launch { _effect.emit(AmrEffect.NavigateToAmrDetail(intent.amrId)) }
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
                    _state.value = _state.value.copy(
                        fullAmrList = list,
                        amrList = filitered,
                        isLoading = false,
                        error = null
                    )
                } catch (e: Exception) {
                    _effect.emit(AmrEffect.ShowError("Amr 리스트 로드 실패: ${e.message}"))
                }
                delay(intervalMs)
            }
        }
    }

    fun refreshAmrList() {
        viewModelScope.launch {
            try {
                val list = getAmrListUseCase()
                val filtered = filterList(list, _state.value.selectedCategory)
                _state.value = _state.value.copy(
                    fullAmrList = list,
                    amrList = filtered,
                    isLoading = false,
                    error = null
                )
            } catch (e: Exception) {
                _effect.emit(AmrEffect.ShowError("AMR 리스트 로드 실패: ${e.message}"))
            }
        }
    }

    private fun filterList(list: List<AmrUiModel>, category: AmrCategory): List<AmrUiModel> {
        return when (category) {
            AmrCategory.ALL -> list
            AmrCategory.RUNNING -> list.filter { it.status == AmrStatus.RUNNING }
            AmrCategory.CHARGING -> list.filter { it.status == AmrStatus.CHARGING }
            AmrCategory.CHECK -> list.filter { it.status == AmrStatus.CHECK }
        }

    }
}
