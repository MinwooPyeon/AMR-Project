package com.android.ssamr.main

import androidx.compose.foundation.layout.padding
import androidx.compose.material3.Scaffold
import androidx.compose.runtime.Composable
import androidx.compose.runtime.derivedStateOf
import androidx.compose.runtime.getValue
import androidx.compose.runtime.remember
import androidx.compose.ui.Modifier
import androidx.navigation.compose.NavHost
import androidx.navigation.compose.composable
import androidx.navigation.compose.currentBackStackEntryAsState
import androidx.navigation.compose.rememberNavController
import com.android.ssamr.core.ui.SSAMRBottomBar
import com.android.ssamr.core.ui.SSAMRCustomTopAppBar
import com.android.ssamr.core.ui.SSAMRTopAppBar
import com.android.ssamr.feature.amr.AmrManageRoute
import com.android.ssamr.main.navigation.AlarmScreen
import com.android.ssamr.main.navigation.AmrScreen
import com.android.ssamr.main.navigation.DashboardScreen
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
            bottomNavScreens.any { it.route == currentRoute }
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
            composable(DashboardScreen.route) { /* DashboardScreen() */ }
            composable(AmrScreen.route) { AmrManageRoute() }
            composable(AlarmScreen.route) { /* AlarmScreen() */ }
            composable(MoreScreen.route) { /* MoreScreen() */ }
        }
    }
}
