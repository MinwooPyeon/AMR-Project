package com.android.ssamr.feature.amrWebcam

import android.app.Activity
import android.content.pm.ActivityInfo
import android.util.Log
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.padding
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.Close
import androidx.compose.material.icons.filled.Star
import androidx.compose.material3.Icon
import androidx.compose.material3.IconButton
import androidx.compose.runtime.Composable
import androidx.compose.runtime.DisposableEffect
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.platform.LocalLifecycleOwner
import androidx.compose.ui.res.painterResource
import androidx.compose.ui.tooling.preview.Preview
import androidx.compose.ui.unit.dp
import androidx.lifecycle.Lifecycle
import androidx.lifecycle.LifecycleEventObserver
import com.android.ssamr.R
import com.android.ssamr.ui.theme.SSAMRTheme
import com.google.accompanist.systemuicontroller.rememberSystemUiController

@Composable
fun AmrWebcamScreen(
    state: AmrWebcamState,
    sendIntent: (AmrWebcamIntent) -> Unit,
    onFullScreenChanged: (Boolean) -> Unit = {},
) {
    val context = LocalContext.current
    val activity = context as? Activity

    var isFullScreen by remember { mutableStateOf(false) }
    val systemUiController = rememberSystemUiController()

    // 전체화면 상태가 바뀔 때만 orientation을 바꿈
    LaunchedEffect(isFullScreen) {
        onFullScreenChanged(isFullScreen)
        if (isFullScreen) {
            activity?.requestedOrientation = ActivityInfo.SCREEN_ORIENTATION_LANDSCAPE
            systemUiController.isSystemBarsVisible = false
            Log.d("WEB", "isFullScreen = $isFullScreen")
        } else {
            activity?.requestedOrientation = ActivityInfo.SCREEN_ORIENTATION_UNSPECIFIED
            systemUiController.isSystemBarsVisible = true
            Log.d("WEB", "isFullScreen = $isFullScreen")
        }
    }

    // 이 route가 완전히 dispose될 때만 orientation을 원복
    DisposableEffect(Unit) {
        Log.d("WEB", "DisposableEffect enter")
        onDispose {
            activity?.requestedOrientation = ActivityInfo.SCREEN_ORIENTATION_UNSPECIFIED
            systemUiController.isSystemBarsVisible = true
            Log.d("WEB", "DisposableEffect dispose")
        }
    }

    Box(modifier = Modifier.fillMaxSize()) {
        RtspPlayerView(
            url = state.rtspUrl,
            modifier = Modifier.fillMaxSize()
        )

        if (!isFullScreen) {
            AmrWebcamInfoPanel(
                state = state,
                modifier = Modifier
                    .align(Alignment.TopCenter)
                    .padding(16.dp)
            )
        }
        IconButton(
            onClick = {
//                onToggleFullScreen?.invoke()
                isFullScreen = !isFullScreen
                      },
            modifier = Modifier
                .align(Alignment.TopEnd)
                .padding(20.dp)
        ) {
            Icon(
                painter = painterResource(R.drawable.ic_fullscreen),
                contentDescription = if (isFullScreen) "전체화면 종료" else "전체화면",
                tint = androidx.compose.ui.graphics.Color.White
            )
        }
    }
}


@Preview
@Composable
fun ArmWebcamScreenPreview() {
    SSAMRTheme {
        AmrWebcamScreen(
            state = AmrWebcamState(),
            sendIntent = {},
        )
    }
}