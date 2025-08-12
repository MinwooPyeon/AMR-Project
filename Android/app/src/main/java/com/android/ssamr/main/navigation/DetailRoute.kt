package com.android.ssamr.main.navigation

import java.net.URLEncoder
import java.nio.charset.StandardCharsets

sealed interface DetailRoute {
    val route: String
    val label: String
}

data object AmrDetailScreen : DetailRoute {
    override val route = "amr_detail"
    override val label = "AMR 상세"

    fun routeWithArgs(serial: String) = "$route/$serial"
}

data object FullmapRoute : DetailRoute {
    override val route = "full_map"
    override val label = "전체 지도"
}

data object WebcamScreen: DetailRoute {
    override val route = "amr_webcam"
    override val label = "Amr 웹캠"
}

data object NotificationDetailScreen : DetailRoute {
    override val route = "notification_detail"
    override val label = "알림 상세"
}

data object FullscreenPhotoScreen : DetailRoute {
    override val route = "photo_view"
    override val label = "사진 보기"

    fun routeWithArgs(imageUrl: String): String {
        val encoded = URLEncoder.encode(imageUrl, StandardCharsets.UTF_8.toString())
        return "$route/$encoded"
    }

}