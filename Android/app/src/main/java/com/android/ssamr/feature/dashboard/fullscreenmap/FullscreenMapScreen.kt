// FullscreenMapScreen.kt
package com.android.ssamr.feature.dashboard.fullscreenmap

import androidx.compose.foundation.layout.Box
import androidx.compose.material3.CircularProgressIndicator
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier

@Composable
fun FullscreenMapScreen(
    state: FullscreenMapState,
    sendIntent: (FullscreenMapIntent) -> Unit
) {
    Box(modifier = Modifier) {
        when {
            state.isLoading -> {
                CircularProgressIndicator(modifier = Modifier.align(Alignment.Center))
            }
            state.mapImage == null -> {
                Text("지도를 불러오는 중...", modifier = Modifier.align(Alignment.Center))
            }
            else -> {
                FullscreenMapContent(
                    state = state,
                    sendIntent = sendIntent
                )
            }
        }
    }
}
