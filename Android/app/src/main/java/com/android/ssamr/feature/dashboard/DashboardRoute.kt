package com.android.ssamr.feature.dashboard

import androidx.compose.runtime.*
import androidx.compose.runtime.collectAsState
import androidx.compose.runtime.LaunchedEffect
import androidx.hilt.navigation.compose.hiltViewModel
import kotlinx.coroutines.flow.collectLatest

@Composable
fun DashboardRoute(
    viewModel: DashboardViewModel = hiltViewModel(),
    navigateToAmrDetail: (Long) -> Unit,
    navigateToMapFullScreen: () -> Unit,
    navigateToAmrList: () -> Unit,
    showSnackbar: (String) -> Unit
) {
    val state by viewModel.state.collectAsState()

    LaunchedEffect(Unit) {
        viewModel.effect.collectLatest { effect ->
            when (effect) {
                is DashboardEffect.NavigateToAmrDetail -> navigateToAmrDetail(effect.amrId)
                is DashboardEffect.NavigateToMapFullScreen -> navigateToMapFullScreen()
                is DashboardEffect.NavigateToAmrList -> navigateToAmrList()
                is DashboardEffect.ShowError -> showSnackbar(effect.message)
            }
        }
    }

    DashboardContent(
        state = state,
        onIntent = { viewModel.onIntent(it) }
    )
}