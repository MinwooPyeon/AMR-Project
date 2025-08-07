package com.android.ssamr.main.navigation

import androidx.compose.material3.Icon
import androidx.compose.material3.IconButton
import androidx.compose.ui.res.painterResource
import androidx.navigation.NavHostController
import com.android.ssamr.R
import com.android.ssamr.core.ui.TopBarConfig

fun getTopBarConfig(
    route: String,
    navController: NavHostController,
    onCallback: (() -> Unit)? = null
): TopBarConfig? {
    return when {
        route == DashboardScreen.route -> TopBarConfig(
            title = "SafetyBot",
            subTitle = "물류 공장 관제 시스템",
            actions = {
                IconButton(onClick = {}) {
                    Icon(painterResource(R.drawable.ic_notification), null)
                }
                IconButton(onClick = {}) {
                    Icon(painterResource(R.drawable.ic_call), null)
                }
            },
            isCustom = true
        )

        route == AmrScreen.route -> TopBarConfig(
            title = "AMR 관리",
            actions = {
                IconButton(onClick = { onCallback?.invoke() }) {
                    Icon(painterResource(R.drawable.ic_refresh), null)
                }
            },
            showBack = true,
            onBackClick = {
                navController.navigate(DashboardScreen.route) {
                    popUpTo(DashboardScreen.route) { inclusive = false }
                    launchSingleTop = true
                }
            })

        route == AlarmScreen.route -> TopBarConfig(
            title = "알림",
            actions = {
                IconButton(onClick = {}) {
                    Icon(painterResource(R.drawable.ic_call), null)
                }
            },
            showBack = true,
            onBackClick = {
                navController.navigate(DashboardScreen.route) {
                    popUpTo(DashboardScreen.route) { inclusive = false }
                    launchSingleTop = true
                }
            })

        route == MoreScreen.route -> TopBarConfig(
            title = "더보기",
            showBack = true,
            onBackClick = {
                navController.navigate(DashboardScreen.route) {
                    popUpTo(DashboardScreen.route) { inclusive = false }
                    launchSingleTop = true
                }
            })

        route.startsWith("amr_detail") -> TopBarConfig(
            title = "AMR 상세",
            showBack = true,
            onBackClick = { navController.popBackStack() }
        )

        route.startsWith("amr_webcam") -> TopBarConfig(
            title = "AMR 웹캠",
            showBack = true,
            onBackClick = { navController.popBackStack() }
        )

        // ... 기타 화면
        else -> null
    }
}
