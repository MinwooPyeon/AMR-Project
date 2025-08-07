package com.android.ssamr.feature.more

import androidx.compose.runtime.Composable
import androidx.compose.runtime.LaunchedEffect
import androidx.lifecycle.viewmodel.compose.viewModel
import androidx.compose.runtime.collectAsState
import androidx.compose.runtime.getValue
import androidx.navigation.NavController

@Composable
fun MorescreenScreen(
    navController: NavController,
    viewModel: MorescreenViewModel = viewModel()
) {
    val state by viewModel.state.collectAsState()

    // 최초 1회 사용자 정보 불러오기
    LaunchedEffect(Unit) {
        viewModel.loadUserProfile()

        // effect 처리 예: Navigation
        viewModel.effect.collect { effect ->
            when (effect) {
                is MorescreenEffect.NavigateTo -> {
                    when (effect.destination) {
                        MorescreenDestination.PROFILE -> {
                            navController.navigate("editProfile")
                        }
                        MorescreenDestination.SETTING -> {
                            navController.navigate("setting")
                        }
                        MorescreenDestination.HELP -> {
                            navController.navigate("help")
                        }
                        MorescreenDestination.NOTICE -> {
                            navController.navigate("notice")
                        }
                        MorescreenDestination.VERSION_INFO -> {
                            navController.navigate("versionInfo")
                        }
                    }
                }
            }
        }
    }

    MorescreenContent(
        state = state,
        onIntent = viewModel::onIntent
    )
}
