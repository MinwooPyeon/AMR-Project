package com.android.ssamr.feature.report

import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.android.ssamr.feature.report.ReportEffect.*
import dagger.hilt.android.lifecycle.HiltViewModel
import kotlinx.coroutines.flow.MutableSharedFlow
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.SharedFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.launch
import javax.inject.Inject

@HiltViewModel
class ReportViewModel @Inject constructor(
//    private val getReportListUseCase: GetReportListUseCase
) : ViewModel() {

    private val _state = MutableStateFlow(ReportState())
    val state: StateFlow<ReportState> = _state.asStateFlow()

    private val _effect = MutableSharedFlow<ReportEffect>()
    val effect: SharedFlow<ReportEffect>
        get() = _effect

    fun sendIntent(intent: ReportIntent) {
        when (intent) {
            is ReportIntent.ClickReportCard -> {
                viewModelScope.launch { _effect.emit(NavigateToReportDetail(intent.reportId)) }
            }

            is ReportIntent.ClickReportCategory -> TODO()
        }

    }
}