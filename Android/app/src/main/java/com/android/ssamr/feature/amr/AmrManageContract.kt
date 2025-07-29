package com.android.ssamr.feature.amr

// core.domain로 이동 에정
data class AmrUiModel(
    val id: Long,
    val name: String,
    val status: AmrStatus,
    val location: String,
    val speed: String,
    val job: String,
    val battery: Int,
)

data class AmrState(
    val selectedCategory: AmrCategory = AmrCategory.ALL,
    val fullAmrList: List<AmrUiModel> = emptyList(),
    val amrList: List<AmrUiModel> = emptyList(),
    val categoryCounts: Map<AmrCategory, Int> = emptyMap(),
    val isLoading: Boolean = false,
    val error: String? = null
)

sealed class AmrIntent {
    data class ClickAmrCategory(val category: AmrCategory) : AmrIntent()

    data class ClickAmrManageCard(val amrId: Long) : AmrIntent()

}

sealed class AmrEffect {
    data class NavigateToAmrDetail(val amrId: Long) : AmrEffect()
    data class ShowError(val message: String) : AmrEffect()
}

enum class AmrCategory(val label: String) {
    ALL("전체"),
    RUNNING("작동중"),
    CHARGING("충전중"),
    CHECK("점검")
}

enum class AmrStatus(val display: String) {
    RUNNING("작동중"),
    CHARGING("충전중"),
    CHECK("점검중")
}