package com.android.ssamr.feature.amr

import com.android.ssamr.core.domain.model.AmrCategory
import com.android.ssamr.core.domain.model.AmrStatus

data class AmrState(
    val selectedCategory: AmrCategory = AmrCategory.ALL,
    val fullAmrList: List<AmrStatus> = emptyList(),
    val amrList: List<AmrStatus> = emptyList(),
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