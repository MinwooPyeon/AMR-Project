package com.android.ssamr.main

import androidx.compose.foundation.background
import androidx.compose.foundation.layout.padding
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Scaffold
import androidx.compose.runtime.Composable
import androidx.compose.runtime.derivedStateOf
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.navigation.NavType
import androidx.navigation.compose.NavHost
import androidx.navigation.compose.composable
import androidx.navigation.compose.currentBackStackEntryAsState
import androidx.navigation.compose.rememberNavController
import androidx.navigation.navArgument
import com.android.ssamr.core.ui.SSAMRBottomBar
import com.android.ssamr.core.ui.SSAMRCustomTopAppBar
import com.android.ssamr.core.ui.SSAMRTopAppBar
import com.android.ssamr.feature.amr.AmrManageRoute
import com.android.ssamr.feature.amrDetail.AmrDetailRoute
import com.android.ssamr.feature.dashboard.DashboardRoute
import com.android.ssamr.feature.dashboard.fullscreenmap.FullscreenMapRoute
import com.android.ssamr.feature.more.MorescreenRoute
import com.android.ssamr.feature.amrWebcam.AmrWebcamRoute
import com.android.ssamr.feature.notification.NotificationRoute
import com.android.ssamr.feature.notificationDetail.NotificationDetailRoute
import com.android.ssamr.feature.notificationDetail.fullscreenPhoto.FullscreenPhotoRoute
import com.android.ssamr.feature.report.ReportRoute
import com.android.ssamr.feature.reportDetail.ReportDetailRoute
import com.android.ssamr.main.navigation.AlarmScreen
import com.android.ssamr.main.navigation.AmrDetailScreen
import com.android.ssamr.main.navigation.AmrScreen
import com.android.ssamr.main.navigation.DashboardScreen
import com.android.ssamr.main.navigation.getBaseRoute
import com.android.ssamr.main.navigation.FullmapRoute
import com.android.ssamr.main.navigation.FullscreenPhotoScreen
import com.android.ssamr.main.navigation.topBarPolicies
import com.android.ssamr.main.navigation.MoreScreen
import com.android.ssamr.main.navigation.NotificationDetailScreen
import com.android.ssamr.main.navigation.ReportDetailScreen
import com.android.ssamr.main.navigation.ReportScreen
import com.android.ssamr.main.navigation.WebcamScreen
import com.android.ssamr.main.navigation.bottomNavScreens
import com.android.ssamr.main.navigation.getTopBarConfig
import com.android.ssamr.ui.theme.BackgroundColor

@Composable
fun MainScreen() {
    val navController = rememberNavController()
    val navBackStackEntry by navController.currentBackStackEntryAsState()
    val currentRoute = navBackStackEntry?.destination?.route

    var isWebcamFullScreen by remember { mutableStateOf(false) }

    val showTopBar by remember(currentRoute, isWebcamFullScreen) {
        derivedStateOf {
//            currentRoute != "login" && !(currentRoute?.startsWith("amr_webcam") == true && isWebcamFullScreen)
            val baseRoute = getBaseRoute(currentRoute)
            topBarPolicies[baseRoute]?.invoke(isWebcamFullScreen) ?: true
        }
    }
    val shouldShowBottomBar by remember(currentRoute) {
        derivedStateOf {
            bottomNavScreens.any { it.route == currentRoute } || currentRoute == "full_map"
        }
    }
    var onCallbackAction: (() -> Unit)? by remember { mutableStateOf(null) }

    var onRefreshAction: (() -> Unit)? by remember { mutableStateOf(null) }

    val topBarConfig =
        currentRoute?.let { getTopBarConfig(it, navController, onCallback = onCallbackAction, onRefresh = onRefreshAction) }

    Scaffold(
        containerColor = BackgroundColor,
        topBar = {
            if (showTopBar && topBarConfig != null) {
                if (topBarConfig.isCustom) {
                    SSAMRCustomTopAppBar(
                        title = topBarConfig.title,
                        subTitle = topBarConfig.subTitle,
                        showBack = topBarConfig.showBack,
                        onBackClick = topBarConfig.onBackClick,
                        actions = topBarConfig.actions
                    )
                } else {
                    SSAMRTopAppBar(
                        title = topBarConfig.title,
                        showBack = topBarConfig.showBack,
                        onBackClick = topBarConfig.onBackClick,
                        actions = topBarConfig.actions
                    )
                }
            }
        },
        bottomBar = {
            if (shouldShowBottomBar && currentRoute != null) {
                SSAMRBottomBar(
                    selectedRoute = currentRoute,
                    onTabSelected = { route ->
                        if (route != currentRoute) {
                            navController.navigate(route) {
                                launchSingleTop = true
                                restoreState = true
                                popUpTo(navController.graph.startDestinationId) { saveState = true }
                            }
                        }
                    }
                )
            }
        }
    ) { innerPadding ->
        NavHost(
            navController = navController,
            startDestination = DashboardScreen.route,
            modifier = Modifier
                .padding(innerPadding)
        ) {
            composable(DashboardScreen.route) {
                DashboardRoute(
                    navigateToAmrDetail = { serial -> navController.navigate("amr_detail/$serial") },
                    navigateToMapFullScreen = { navController.navigate("full_map") },
                    navigateToAmrList = {
                        navController.navigate("amr") {
                            popUpTo(navController.graph.startDestinationId) { saveState = true }
                            launchSingleTop = true
                            restoreState = true
                        }
                    }
                )
            }
            composable(AmrScreen.route) {
                AmrManageRoute(
                    navigateToAmrDetail = { serial ->
                        navController.navigate("amr_detail/$serial")
                    },
                    onRefresh = { onCallbackAction = it }
                )
            }
            composable(AlarmScreen.route) {
                NotificationRoute(
                    navigateToNotificationDetail = { notificationId ->
                        navController.navigate("notification_detail/$notificationId")
                    },
                    onRefresh = { setter -> onRefreshAction = setter },
                    onCallClick = { onCallbackAction = it }
                )
            }
            composable(MoreScreen.route) {
                MorescreenRoute(
                    navController = navController, // 상위 NavController
                    navigateToEditProfile = { navController.navigate("editProfile") },
                    navigateToSetting = { navController.navigate("setting") },
                    navigateToHelp = { navController.navigate("help") },
                    navigateToNotice = { navController.navigate("notice") },
                    navigateToVersionInfo = { navController.navigate("versionInfo") },
                    navigateToReport = {navController.navigate("report")}
                )
            }
            composable(FullmapRoute.route) {
                FullscreenMapRoute(
                    navigateToAmrDetail = { serial ->
                        navController.navigate(AmrDetailScreen.routeWithArgs(serial))
                    },
                    onBack = { navController.popBackStack() }
                )
            }
            composable(
                route = "${AmrDetailScreen.route}/{serial}",
                arguments = listOf(
                    navArgument("serial") { type = NavType.StringType },
                )
            ) { backStackEntry ->
                val serial = backStackEntry.arguments?.getString("serial") ?: ""
                AmrDetailRoute(
                    onBack = { navController.popBackStack() },
                    navigateToWebcam = { ipAddress ->
                        navController.navigate("amr_webcam/$serial/$ipAddress")
                    }
                )
            }
            composable(
                route = "${WebcamScreen.route}/{serial}/{ipAddress}",
                arguments = listOf(
                    navArgument("serial") { type = NavType.StringType },
                    navArgument("ipAddress") { type = NavType.StringType }
                )
            ) {
                AmrWebcamRoute(
                    onFullScreenChanged = { isFullScreen ->
                        isWebcamFullScreen = isFullScreen
                    }
                )
            }

            composable(
                route = "${NotificationDetailScreen.route}/{notificationId}",
                arguments = listOf(navArgument("notificationId") { type = NavType.LongType })
            ) {
                NotificationDetailRoute(
                    navigateToPhotoView = { url ->
                        navController.navigate(FullscreenPhotoScreen.routeWithArgs(url))
                    },
                    onBack = { navController.popBackStack() })
            }

            composable(
                route = "${FullscreenPhotoScreen.route}/{imageUrl}",
                arguments = listOf(navArgument("imageUrl") { type = NavType.StringType })
            ) { backStackEntry ->
                val url = backStackEntry.arguments?.getString("imageUrl") ?: ""
                FullscreenPhotoRoute(
                    imageUrl = url,
                    onBack = { navController.popBackStack() }
                )
            }
            composable(ReportScreen.route) {
                ReportRoute(
                    navigateToReportDetail = { id ->
                        navController.navigate("report_detail/$id") },
                    onBack = { navController.popBackStack() },
                )
            }
            composable(
                route = "${ReportDetailScreen.route}/{id}",
                arguments = listOf(navArgument("id") { type = NavType.LongType })
                ) { backStackEntry ->
                    val id = backStackEntry.arguments?.getLong("id") ?: 0L
                    ReportDetailRoute(
                        id = id,
                        onBack = { navController.popBackStack() }
                    )
                }
        }
    }
}