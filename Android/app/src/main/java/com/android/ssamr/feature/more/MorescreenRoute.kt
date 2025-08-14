package com.android.ssamr.feature.more

import androidx.compose.runtime.*
import androidx.compose.runtime.collectAsState
import androidx.compose.runtime.LaunchedEffect
import androidx.hilt.navigation.compose.hiltViewModel
import androidx.navigation.NavController
import kotlinx.coroutines.flow.collectLatest

@Composable
fun MorescreenRoute(
    viewModel: MorescreenViewModel = hiltViewModel(),
    navController: NavController,
    navigateToEditProfile: () -> Unit,
    navigateToSetting: () -> Unit,
    navigateToHelp: () -> Unit,
    navigateToNotice: () -> Unit,
    navigateToVersionInfo: () -> Unit,
    navigateToReport: () -> Unit
) {
    val state by viewModel.state.collectAsState()

    LaunchedEffect(Unit) {
        viewModel.loadUserProfile()

        viewModel.effect.collectLatest { effect ->
            when (effect) {
                is MorescreenEffect.NavigateTo -> {
                    when (effect.destination) {
                        MorescreenDestination.PROFILE -> navigateToEditProfile()
                        MorescreenDestination.SETTING -> navigateToSetting()
                        MorescreenDestination.HELP -> navigateToHelp()
                        MorescreenDestination.NOTICE -> navigateToNotice()
                        MorescreenDestination.VERSION_INFO -> navigateToVersionInfo()
                        MorescreenDestination.REPORT -> navigateToReport()
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
