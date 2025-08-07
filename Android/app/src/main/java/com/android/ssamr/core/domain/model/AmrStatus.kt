package com.android.ssamr.core.domain.model

data class AmrStatus (
    val id: Long,
    val name: String,
    val state: AmrAction,
    val locationX: Double,
    val locationY: Double,
    val speed: String,
    val job: String,
)

enum class AmrCategory(val label: String) {
    ALL("전체"),
    RUNNING("작동중"),
    CHARGING("충전중"),
    CHECKING("점검")
}

enum class AmrAction(val display: String) {
    RUNNING("작동중"),
    CHARGING("충전중"),
    CHECKING("점검중")
}
