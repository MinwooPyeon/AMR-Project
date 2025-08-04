package com.android.ssamr.main

import androidx.compose.foundation.layout.padding
import androidx.compose.material3.Scaffold
import androidx.compose.runtime.Composable
import androidx.compose.runtime.derivedStateOf
import androidx.compose.runtime.getValue
import androidx.compose.runtime.remember
import androidx.compose.ui.Modifier
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
import com.android.ssamr.main.navigation.AlarmScreen
import com.android.ssamr.main.navigation.AmrDetailScreen
import com.android.ssamr.main.navigation.AmrScreen
import com.android.ssamr.main.navigation.DashboardScreen
import com.android.ssamr.main.navigation.FullmapRoute
import com.android.ssamr.main.navigation.MoreScreen
import com.android.ssamr.main.navigation.bottomNavScreens
import com.android.ssamr.main.navigation.getTopBarConfig

@Composable
fun MainScreen() {
    val navController = rememberNavController()
    val navBackStackEntry by navController.currentBackStackEntryAsState()
    val currentRoute = navBackStackEntry?.destination?.route

    val showTopBar by remember(currentRoute) {
        derivedStateOf { currentRoute != "login" }
    }
    val shouldShowBottomBar by remember(currentRoute) {
        derivedStateOf {
            bottomNavScreens.any { it.route == currentRoute } || currentRoute == "full_map"
        }
    }
    val topBarConfig = currentRoute?.let { getTopBarConfig(it, navController) }

    Scaffold(
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
            modifier = Modifier.padding(innerPadding)
        ) {
            composable(DashboardScreen.route) {
                DashboardRoute(
                    navigateToAmrDetail = { amrId -> navController.navigate("amr_detail/$amrId") },
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
                    navigateToAmrDetail = { amrId ->
                        navController.navigate("amr_detail/$amrId")
                    }
                )
            }
            composable(AlarmScreen.route) { /* AlarmScreen() */ }
            composable(MoreScreen.route) {
                MorescreenRoute(
                    navController = navController, // 상위 NavController
                    navigateToEditProfile = { navController.navigate("editProfile") },
                    navigateToSetting = { navController.navigate("setting") },
                    navigateToHelp = { navController.navigate("help") },
                    navigateToNotice = { navController.navigate("notice") },
                    navigateToVersionInfo = { navController.navigate("versionInfo") }
                )
            }
            composable(FullmapRoute.route) {
                FullscreenMapRoute(
                    navigateToAmrDetail = { amrId ->
                        navController.navigate(AmrDetailScreen.routeWithArgs(amrId))
                    },
                    onBack = { navController.popBackStack() }
                )
            }
            composable(
                route = "${AmrDetailScreen.route}/{amrId}",
                arguments = listOf(navArgument("amrId") { type = NavType.LongType })
            ) { backStackEntry ->
                val amrId = backStackEntry.arguments?.getLong("amrId") ?: 0L
                AmrDetailRoute(
                    onBack = { navController.popBackStack() }
                )
            }
        }
    }
}