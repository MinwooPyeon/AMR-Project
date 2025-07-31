// FullscreenMapRoute.kt
package com.android.ssamr.feature.dashboard.fullscreenmap

import androidx.compose.material3.SnackbarHostState
import androidx.compose.runtime.*
import androidx.compose.runtime.collectAsState
import androidx.hilt.navigation.compose.hiltViewModel
import kotlinx.coroutines.flow.collectLatest
import kotlinx.coroutines.launch

@Composable
fun FullscreenMapRoute(
    viewModel: FullscreenMapViewModel = hiltViewModel(),
    navigateToAmrDetail: (Long) -> Unit,
    onBack: () -> Unit
) {
    val state by viewModel.state.collectAsState()
    val coroutineScope = rememberCoroutineScope()
    val snackbarHostState = remember { SnackbarHostState() }

    // Intent
    LaunchedEffect(Unit) {
        viewModel.onIntent(FullscreenMapIntent.LoadMap)
    }

    // Effect 처리
    LaunchedEffect(Unit) {
        viewModel.effect.collectLatest { effect ->
            when (effect) {
                is FullscreenMapEffect.ShowError -> {
                    coroutineScope.launch {
                        snackbarHostState.showSnackbar(effect.message)
                    }
                }
                is FullscreenMapEffect.ShowAmrDetail -> navigateToAmrDetail(effect.amrId)
                FullscreenMapEffect.NavigateBack -> onBack()
            }
        }
    }

    FullscreenMapScreen(
        state = state,
        sendIntent = viewModel::onIntent
    )
}
