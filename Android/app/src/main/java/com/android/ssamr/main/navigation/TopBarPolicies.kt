package com.android.ssamr.main.navigation

val topBarPolicies: Map<String, (Boolean) -> Boolean> = mapOf(
    "login" to { _ -> false },
    DashboardScreen.route to { _ -> true },
    AmrScreen.route to { _ -> true },
    AlarmScreen.route to { _ -> true },
    MoreScreen.route to { _ -> true },
    WebcamScreen.route to { isFullScreen -> !isFullScreen }
)

fun getBaseRoute(route: String?): String? = route?.substringBefore("/")