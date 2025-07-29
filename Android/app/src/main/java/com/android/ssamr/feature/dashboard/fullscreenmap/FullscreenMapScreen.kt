package com.android.ssamr.feature.dashboard.fullscreenmap

import androidx.compose.material3.SnackbarHostState
import androidx.compose.runtime.*
import androidx.compose.runtime.collectAsState
import androidx.hilt.navigation.compose.hiltViewModel
import kotlinx.coroutines.flow.collectLatest
import kotlinx.coroutines.launch

@Composable
fun FullscreenMapScreen(
    viewModel: FullscreenMapViewModel = hiltViewModel(),
    onBack: () -> Unit,
    navigateToAmrDetail: (Long) -> Unit
) {
    val state by viewModel.state.collectAsState()
    val coroutineScope = rememberCoroutineScope()
    val snackbarHostState = remember { SnackbarHostState() }

    // 👇 여기가 핵심: 초기 Intent 발생
    LaunchedEffect(Unit) {
        viewModel.onIntent(FullscreenMapIntent.LoadMap)
    }

    LaunchedEffect(Unit) {
        viewModel.effect.collectLatest { effect ->
            when (effect) {
                is FullscreenMapEffect.ShowError -> {
                    coroutineScope.launch {
                        snackbarHostState.showSnackbar(effect.message)
                    }
                }
                is FullscreenMapEffect.ShowAmrDetail -> {
                    navigateToAmrDetail(effect.amrId)
                }
                FullscreenMapEffect.NavigateBack -> {
                    onBack()
                }
            }
        }
    }

    FullscreenMapContent(
        state = state,
        sendIntent = viewModel::onIntent
    )
}
