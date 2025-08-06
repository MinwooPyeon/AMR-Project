package com.android.ssamr.main.navigation

sealed interface DetailRoute {
    val route: String
    val label: String
}

data object AmrDetailScreen : DetailRoute {
    override val route = "amr_detail"
    override val label = "AMR 상세"
<<<<<<< HEAD

    fun routeWithArgs(amrId: Long) = "$route/$amrId"
}

data object FullmapRoute : DetailRoute {
    override val route = "full_map"
    override val label = "전체 지도"
=======
}

data object WebcamScreen: DetailRoute {
    override val route = "amr_webcam"
    override val label = "Amr 웹캠"
>>>>>>> origin/develop
}