package com.android.ssamr.feature.notificationDetail

import androidx.compose.runtime.Composable
import androidx.compose.runtime.collectAsState
import androidx.compose.runtime.getValue
import androidx.hilt.navigation.compose.hiltViewModel

@Composable
fun NotificationDetailRoute(
    onBack: () -> Unit,
    viewModel: NotificationDetailViewModel = hiltViewModel()
) {
    val state by viewModel.state.collectAsState()

    NotificationDetailScreen(
        state = state,
        {},
        sendIntent = viewModel::sendIntent
    )
}
