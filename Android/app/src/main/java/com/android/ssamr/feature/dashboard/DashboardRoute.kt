package com.android.ssamr.feature.dashboard

import androidx.compose.runtime.*
import androidx.compose.runtime.collectAsState
import androidx.compose.runtime.LaunchedEffect
import androidx.hilt.navigation.compose.hiltViewModel
import androidx.navigation.NavController
import kotlinx.coroutines.flow.collectLatest

@Composable
fun DashboardRoute(
    viewModel: DashboardViewModel = hiltViewModel(),
    navController: NavController,
    navigateToAmrDetail: (Long) -> Unit,
    navigateToMapFullScreen: () -> Unit,
    showSnackbar: (String) -> Unit
) {
    val state by viewModel.state.collectAsState()

    LaunchedEffect(Unit) {
        viewModel.effect.collectLatest { effect ->
            when (effect) {
                is DashboardEffect.NavigateToAmrDetail -> navigateToAmrDetail(effect.amrId)
                is DashboardEffect.NavigateToMapFullScreen -> navigateToMapFullScreen()
                is DashboardEffect.NavigateToAmrList -> {
                    navController.navigate("amr") {
                        popUpTo(navController.graph.startDestinationId) {
                            saveState = true
                        }
                        launchSingleTop = true
                        restoreState = true
                    }
                }
                is DashboardEffect.ShowError -> showSnackbar(effect.message)
            }
        }
    }

    DashboardScreen(
        state = state,
        sendIntent = viewModel::onIntent
    )
}