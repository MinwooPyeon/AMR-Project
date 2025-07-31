package com.android.ssamr.feature.amrDetail

import android.widget.Toast
import androidx.compose.runtime.Composable
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.collectAsState
import androidx.compose.runtime.getValue
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

//     상세 정보 불러오기 (최초 진입시 1회)
//    LaunchedEffect(Unit) {
//        viewModel.sendIntent(AmrDetailIntent.LoadAmrDetail)
//    }

    // Effect 처리
    LaunchedEffect(Unit) {
        viewModel.effect.collect { effect ->
            when (effect) {
                is AmrDetailEffect.NavigateToWebcam -> navigateToWebcam()
                is AmrDetailEffect.ShowError -> Toast.makeText(
                    context,
                    effect.message,
                    Toast.LENGTH_SHORT
                ).show()

                is AmrDetailEffect.ShowReturnDialog -> {

                }

                is AmrDetailEffect.ShowStartDialog -> {/* TODO: 다이얼로그 띄우기 */
                }
            }
        }
    }

    AmrManageDetailScreen(
        state = state,
        sendIntent = viewModel::sendIntent
    )
}