package com.android.ssamr.feature.dashboard

import androidx.compose.runtime.*
import androidx.compose.runtime.collectAsState
import androidx.compose.runtime.LaunchedEffect
import androidx.hilt.navigation.compose.hiltViewModel
import kotlinx.coroutines.flow.collectLatest

@Composable
fun DashboardRoute(
    viewModel: DashboardViewModel = hiltViewModel(),
    navigateToAmrDetail: (String) -> Unit,
    navigateToMapFullScreen: () -> Unit,
    navigateToAmrList: () -> Unit
) {
    val state by viewModel.state.collectAsState()

    LaunchedEffect(Unit) {
        viewModel.effect.collectLatest { effect ->
            when (effect) {
                is DashboardEffect.NavigateToAmrDetail -> navigateToAmrDetail(effect.serial)
                is DashboardEffect.NavigateToMapFullScreen -> navigateToMapFullScreen()
                is DashboardEffect.NavigateToAmrList -> navigateToAmrList()
            }
        }
    }

    DashboardScreen(
        state = state,
        sendIntent = viewModel::onIntent
    )
}