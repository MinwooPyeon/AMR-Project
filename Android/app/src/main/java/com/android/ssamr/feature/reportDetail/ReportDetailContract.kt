package com.android.ssamr.feature.reportDetail

import com.android.ssamr.core.domain.model.Report

data class ReportDetailState(
    val report: Report? = null,
    val isImageExpanded: Boolean = false,   // ✅ 보기/닫기 토글 상태
    val isLoading: Boolean = false,
    val error: String? = null
)

sealed class ReportDetailIntent {
    data class LoadReport(val reportId: Long) : ReportDetailIntent()
    data object ToggleImage : ReportDetailIntent()            // ✅ 보기↔닫기
}

sealed class ReportDetailEffect {
    data class ShowError(val message: String) : ReportDetailEffect()
    // ✅ 풀스크린 이미지/스트리밍으로 "이동"할 때만 이런 이펙트를 새로 추가하세요.
    // data class NavigateToImageViewer(val imageUrl: String) : ReportDetailEffect()
}