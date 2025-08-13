package com.android.ssamr.feature.report

import com.android.ssamr.core.domain.model.AmrCategory
import com.android.ssamr.core.domain.model.Report
import com.android.ssamr.core.domain.model.ReportCategory

data class ReportState (
    val selectedCategory: ReportCategory = ReportCategory.ALL,
    val fullReportList: List<Report> = emptyList(),
    val reportList: List<Report> = emptyList(),
    val startCardCounts: Map<ReportCategory, Int> = emptyMap(),
    val report: Report? = null,
    val isLoading: Boolean = false,
    val error: String? = null
)

sealed class ReportIntent {
    data class ClickReportCategory(val category: ReportCategory) : ReportIntent()

    data class ClickReportCard(val reportId: Long) : ReportIntent()
}

sealed class ReportEffect{
    data class NavigateToReportDetail(val reportId: Long) : ReportEffect()
    data class ShowError(val message: String) : ReportEffect()
}