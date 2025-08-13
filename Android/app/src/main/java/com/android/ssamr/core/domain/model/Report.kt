package com.android.ssamr.core.domain.model

data class Report(
    val id: Long,
    val title: String,
    val content: String,
    val riskLevel: ReportAction,
    val area: String,
    val case: String,
    val image: String?,
    val serial: String
)


enum class ReportCategory(val label: String) {
    ALL("전체"),
    COLLAPSE("적재"),
    SMOKE("흡연"),
    EQUIPMENT("안전 장비"),
    DANGER("안전 사고"),
}

enum class ReportAction(val display: String) {
    COLLAPSE("적재"),
    SMOKE("흡연"),
    EQUIPMENT("안전 장비"),
    DANGER("안전 사고"),
}