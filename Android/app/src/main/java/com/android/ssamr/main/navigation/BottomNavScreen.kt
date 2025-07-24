package com.android.ssamr.main.navigation

import com.android.ssamr.R

sealed interface BottomNavScreen {
    val route: String
    val label: String
    val iconRes: Int
}

data object DashboardScreen : BottomNavScreen {
    override val route = "dashboard"
    override val label = "대시보드"
    override val iconRes = R.drawable.ic_dashboard
}

data object AmrScreen : BottomNavScreen {
    override val route = "amr"
    override val label = "AMR"
    override val iconRes = R.drawable.ic_amr
}

data object AlarmScreen : BottomNavScreen {
    override val route = "alarm"
    override val label = "알림"
    override val iconRes = R.drawable.ic_notification
}

data object MoreScreen : BottomNavScreen {
    override val route = "more"
    override val label = "더보기"
    override val iconRes = R.drawable.ic_more
}

val bottomNavScreens = listOf(DashboardScreen, AmrScreen, AlarmScreen, MoreScreen)
