package com.android.ssamr.feature.amrWebcam

import androidx.compose.runtime.Composable
import androidx.compose.runtime.collectAsState
import androidx.compose.runtime.getValue
import androidx.hilt.navigation.compose.hiltViewModel

@Composable
fun AmrWebcamRoute(
    viewModel: AmrWebcamViewModel = hiltViewModel()
) {
    val state by viewModel.state.collectAsState()

    AmrWebcamScreen(
        state = state,
        sendIntent = viewModel::sendIntent
    )
}