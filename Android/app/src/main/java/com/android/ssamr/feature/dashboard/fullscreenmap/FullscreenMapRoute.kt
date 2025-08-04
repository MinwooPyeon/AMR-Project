package com.android.ssamr.feature.dashboard.fullscreenmap

import androidx.compose.runtime.Composable
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.collectAsState
import androidx.hilt.navigation.compose.hiltViewModel

@Composable
fun FullscreenMapRoute(
    viewModel: FullscreenMapViewModel = hiltViewModel(),
    navigateToAmrDetail: (Long) -> Unit,
    onBack: () -> Unit
) {
    val state = viewModel.state.collectAsState().value

    LaunchedEffect(Unit) {
        viewModel.onIntent(FullscreenMapIntent.LoadMap)
    }

    if (state.selectedAmrId != null) {
        navigateToAmrDetail(state.selectedAmrId)
    }

    FullscreenMapScreen(
        state = state,
        sendIntent = viewModel::onIntent
    )
}
