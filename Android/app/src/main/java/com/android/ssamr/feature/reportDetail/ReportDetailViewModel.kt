package com.android.ssamr.feature.reportDetail

import android.os.Build
import androidx.annotation.RequiresApi
import androidx.lifecycle.SavedStateHandle
import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.android.ssamr.core.domain.usecase.report.GetReportDetailUseCase
import dagger.hilt.android.lifecycle.HiltViewModel
import kotlinx.coroutines.Job
import kotlinx.coroutines.flow.MutableSharedFlow
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.SharedFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.flow.catch
import kotlinx.coroutines.flow.launchIn
import kotlinx.coroutines.flow.onEach
import kotlinx.coroutines.flow.onStart
import kotlinx.coroutines.flow.update
import kotlinx.coroutines.launch
import javax.inject.Inject

@HiltViewModel
class ReportDetailViewModel @Inject constructor(
    private val getReportDetail: GetReportDetailUseCase,
    savedStateHandle: SavedStateHandle
) : ViewModel() {

    private val _state = MutableStateFlow(ReportDetailState(isLoading = true))
    val state: StateFlow<ReportDetailState> = _state.asStateFlow()

    private val _effect = MutableSharedFlow<ReportDetailEffect>()
    val effect: SharedFlow<ReportDetailEffect> = _effect

    private val id: Long = requireNotNull(savedStateHandle.get<Long>("id")) {
        "id is null in ReportDetailViewModel"
    }

    private var loadJob: Job? = null

    init {
        // 진입 즉시 로드
        sendIntent(ReportDetailIntent.LoadReport(id))
    }

    fun sendIntent(intent: ReportDetailIntent) {
        when (intent) {
            is ReportDetailIntent.LoadReport -> load(intent.reportId)
            ReportDetailIntent.ToggleImage   -> toggleImage()
        }
    }

    private fun load(reportId: Long) {
        loadJob?.cancel()
        loadJob = viewModelScope.launch {
            // suspend fun invoke(id: Long): Flow<Report?>
            getReportDetail(reportId)
                .onStart {
                    _state.update { it.copy(isLoading = true, error = null) }
                }
                .catch { e ->
                    _state.update { it.copy(isLoading = false, error = e.message) }
                    _effect.emit(ReportDetailEffect.ShowError(e.message ?: "알 수 없는 오류"))
                }
                .collect { report ->
                    if (report == null) {
                        _state.update { it.copy(isLoading = false, error = "리포트를 찾을 수 없어요.") }
                    } else {
                        _state.update { it.copy(report = report, isLoading = false, error = null) }
                    }
                }
        }
    }

    private fun toggleImage() {
        _state.update { it.copy(isImageExpanded = !it.isImageExpanded) }
    }
}