package com.android.ssamr.feature.amrWebcam

import android.widget.Toast
import androidx.compose.runtime.Composable
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.collectAsState
import androidx.compose.runtime.getValue
import androidx.compose.ui.platform.LocalContext
import androidx.hilt.navigation.compose.hiltViewModel
import com.android.ssamr.feature.amrDetail.AmrDetailEffect
import kotlinx.coroutines.flow.collect

@Composable
fun AmrWebcamRoute(
    viewModel: AmrWebcamViewModel = hiltViewModel(),
    onFullScreenChanged: (Boolean) -> Unit = {},
) {
    val state by viewModel.state.collectAsState()
    val context = LocalContext.current

    LaunchedEffect(Unit) {
        viewModel.effect.collect {effect ->
            when(effect) {
                is AmrWebcamEffect.ShowError -> Toast.makeText(
                    context,
                    effect.message,
                    Toast.LENGTH_SHORT
                ).show()
            }
        }
    }

    AmrWebcamScreen(
        state = state,
        sendIntent = viewModel::sendIntent,
        onFullScreenChanged = onFullScreenChanged
    )
}