package com.android.ssamr.main.navigation

sealed interface DetailRoute {
    val route: String
    val label: String
}

data object AmrDetailScreen : DetailRoute {
    override val route = "amr_detail"
    override val label = "AMR 상세"
}