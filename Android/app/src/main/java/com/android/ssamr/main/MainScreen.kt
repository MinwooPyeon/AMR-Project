package com.android.ssamr.main

import androidx.compose.foundation.layout.padding
import androidx.compose.material3.Scaffold
import androidx.compose.runtime.Composable
import androidx.compose.runtime.derivedStateOf
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
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
import com.android.ssamr.feature.amrWebcam.AmrWebcamRoute
import com.android.ssamr.main.navigation.AlarmScreen
import com.android.ssamr.main.navigation.AmrDetailScreen
import com.android.ssamr.main.navigation.AmrScreen
import com.android.ssamr.main.navigation.DashboardScreen
import com.android.ssamr.main.navigation.MoreScreen
import com.android.ssamr.main.navigation.WebcamScreen
import com.android.ssamr.main.navigation.bottomNavScreens
import com.android.ssamr.main.navigation.getBaseRoute
import com.android.ssamr.main.navigation.getTopBarConfig
import com.android.ssamr.main.navigation.topBarPolicies

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
            bottomNavScreens.any { it.route == currentRoute }
        }
    }

    var onCallbackAction: (() -> Unit)? by remember { mutableStateOf(null) }

    val topBarConfig =
        currentRoute?.let { getTopBarConfig(it, navController, onCallback = onCallbackAction) }

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
            composable(DashboardScreen.route) { /* DashboardScreen() */ }
            composable(AmrScreen.route) {
                AmrManageRoute(
                    navigateToAmrDetail = { amrId ->
                        navController.navigate("amr_detail/$amrId")
                    },
                    onRefresh = { onCallbackAction = it }
                )
            }
            composable(AlarmScreen.route) { /* AlarmScreen() */ }
            composable(MoreScreen.route) { /* MoreScreen() */ }
            composable(
                route = "${AmrDetailScreen.route}/{amrId}",
                arguments = listOf(navArgument("amrId") { type = NavType.LongType })
            ) { backStackEntry ->
                val amrId = backStackEntry.arguments?.getLong("amrId") ?: 0L
                AmrDetailRoute(
                    onBack = { navController.popBackStack() },
                    navigateToWebcam = { ipAddress ->
                        navController.navigate("amr_webcam/$amrId/$ipAddress")
                    }
                )
            }
            composable(
                route = "${WebcamScreen.route}/{amrId}/{ipAddress}",
                arguments = listOf(
                    navArgument("amrId") { type = NavType.LongType },
                    navArgument("ipAddress") { type = NavType.StringType }
                )
            ) {
                AmrWebcamRoute(
                    onFullScreenChanged = { isFullScreen ->
                        isWebcamFullScreen = isFullScreen
                    }
                )
            }
        }
    }
}
