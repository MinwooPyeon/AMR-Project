package com.android.ssamr.feature.report

import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.android.ssamr.core.domain.model.Report
import com.android.ssamr.core.domain.model.ReportAction
import com.android.ssamr.core.domain.model.ReportCategory
import com.android.ssamr.core.domain.usecase.report.GetReportListUseCase
import com.android.ssamr.feature.report.ReportEffect.*
import dagger.hilt.android.lifecycle.HiltViewModel
import kotlinx.coroutines.flow.MutableSharedFlow
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.SharedFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.flow.catch
import kotlinx.coroutines.flow.launchIn
import kotlinx.coroutines.flow.onEach
import kotlinx.coroutines.flow.update
import kotlinx.coroutines.launch
import javax.inject.Inject

@HiltViewModel
class ReportViewModel @Inject constructor(
    private val getReportListUseCase: GetReportListUseCase
) : ViewModel() {

    private val _state = MutableStateFlow(ReportState(isLoading = true))
    val state: StateFlow<ReportState> = _state.asStateFlow()

    private val _effect = MutableSharedFlow<ReportEffect>()
    val effect: SharedFlow<ReportEffect>
        get() = _effect

    init {
        collectReports()
    }

    private fun collectReports() {
        getReportListUseCase()
            .onEach { reports ->
                val selected = _state.value.selectedCategory
                val counts = buildCategoryCounts(reports)
                val filtered = filterByCategory(reports, selected)
                _state.update {
                    it.copy(
                        fullReportList = reports,
                        reportList = filtered,
                        startCardCounts = counts,
                        isLoading = false,
                        error = null
                    )
                }
            }
            .catch { e ->
                _state.update { it.copy(isLoading = false, error = e.message) }
                _effect.emit(ReportEffect.ShowError(e.message ?: "알 수 없는 오류"))
            }
            .launchIn(viewModelScope)
        }


    fun sendIntent(intent: ReportIntent) {
        when (intent) {
            is ReportIntent.ClickReportCard -> {
                viewModelScope.launch { _effect.emit(NavigateToReportDetail(intent.reportId)) }
            }

            is ReportIntent.ClickReportCategory -> {
                val selected = intent.category
                val filtered = filterByCategory(_state.value.fullReportList, selected)
                _state.update {
                    it.copy(
                        selectedCategory = selected,
                        reportList = filtered
                    )
                }
            }
        }

    }

    private fun filterByCategory(
        list: List<Report>,
        category: ReportCategory
    ): List<Report> {
        if (category == ReportCategory.ALL) return list
        val target = when (category) {
            ReportCategory.COLLAPSE  -> ReportAction.COLLAPSE.name
            ReportCategory.SMOKE     -> ReportAction.SMOKE.name
            ReportCategory.EQUIPMENT -> ReportAction.EQUIPMENT.name
            ReportCategory.DANGER    -> ReportAction.DANGER.name
            ReportCategory.ALL       -> null
        }
        return if (target == null) list else list.filter { it.case == target }

    }
    private fun buildCategoryCounts(list: List<Report>): Map<ReportCategory, Int> {
        fun cnt(a: ReportAction) = list.count { it.case == a.name }
        return mapOf(
            ReportCategory.ALL to list.size,
            ReportCategory.COLLAPSE to cnt(ReportAction.COLLAPSE),
            ReportCategory.SMOKE to cnt(ReportAction.SMOKE),
            ReportCategory.EQUIPMENT to cnt(ReportAction.EQUIPMENT),
            ReportCategory.DANGER to cnt(ReportAction.DANGER)
        )
    }
}

