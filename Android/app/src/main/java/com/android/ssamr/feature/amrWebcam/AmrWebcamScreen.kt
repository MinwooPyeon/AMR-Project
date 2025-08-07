package com.android.ssamr.feature.amrWebcam

import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.padding
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.tooling.preview.Preview
import androidx.compose.ui.unit.dp
import com.android.ssamr.ui.theme.SSAMRTheme

@Composable
fun AmrWebcamScreen(
    state: AmrWebcamState,
    sendIntent: (AmrWebcamIntent) -> Unit
) {
    Box(modifier = Modifier.fillMaxSize()) {
        RtspPlayerView(
            url = state.rtspUrl,
            modifier = Modifier.fillMaxSize()
        )

        AmrWebcamInfoPanel(
            state = state,
            modifier = Modifier
                .align(Alignment.TopCenter)
                .padding(16.dp)
        )
    }
}

@Preview
@Composable
fun ArmWebcamScreenPreview() {
    SSAMRTheme {
        AmrWebcamScreen(
            state = AmrWebcamState()

        ) { }
    }
}