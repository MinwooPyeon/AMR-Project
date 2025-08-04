package com.android.ssamr.feature.dashboard.fullscreenmap

import androidx.compose.foundation.layout.*
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.unit.dp

@Composable
fun FullscreenMapScreen(
    state: FullscreenMapState,
    sendIntent: (FullscreenMapIntent) -> Unit
) {
    Box(modifier = Modifier.fillMaxSize()) {
        when {
            state.isLoading -> {
                androidx.compose.material3.CircularProgressIndicator(
                    modifier = Modifier.align(Alignment.Center)
                )
            }
            state.mapImage == null -> {
                Text(
                    "지도를 불러오는 중...",
                    modifier = Modifier.align(Alignment.Center),
                    color = Color.Gray
                )
            }
            else -> {
                FullscreenMapContent(state = state, sendIntent = sendIntent)
            }
        }

        state.error?.let { errorMessage ->
            Text(
                text = errorMessage,
                color = Color.Red,
                modifier = Modifier
                    .align(Alignment.BottomCenter)
                    .padding(16.dp)
            )
        }
    }
}
