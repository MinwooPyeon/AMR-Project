package com.android.ssamr.feature.amrDetail

import android.widget.Toast
import androidx.compose.foundation.layout.padding
import androidx.compose.runtime.*
import androidx.compose.ui.Modifier
import androidx.compose.ui.platform.LocalContext
import androidx.hilt.navigation.compose.hiltViewModel

@Composable
fun AmrDetailRoute(
    viewModel: AmrDetailViewModel = hiltViewModel(),
    onBack: () -> Unit = {},
    navigateToWebcam: () -> Unit = {}
) {
    val state by viewModel.state.collectAsState()
    val context = LocalContext.current

    // Effect 처리
    LaunchedEffect(Unit) {
        viewModel.effect.collect { effect ->
            when (effect) {
                is AmrDetailEffect.NavigateToWebcam -> navigateToWebcam()
                is AmrDetailEffect.ShowError -> {
                    Toast.makeText(context, effect.message, Toast.LENGTH_SHORT).show()
                }
                is AmrDetailEffect.ShowSuccess -> {
                    Toast.makeText(context, effect.message, Toast.LENGTH_SHORT).show()
                }
                is AmrDetailEffect.ShowStartDialog -> {} // 필요시 사용
                is AmrDetailEffect.ShowReturnDialog -> {}
            }
        }
    }

    AmrManageDetailScreen(
        state = state,
        sendIntent = viewModel::sendIntent,
        modifier = Modifier.padding() // 필요시 Scaffold로 감싸고 padding 처리
    )
}
