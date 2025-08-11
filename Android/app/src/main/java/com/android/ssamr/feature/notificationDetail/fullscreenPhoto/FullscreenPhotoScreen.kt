package com.android.ssamr.feature.notificationDetail.fullscreenPhoto

import androidx.activity.compose.BackHandler
import androidx.compose.foundation.background
import androidx.compose.foundation.gestures.detectTapGestures
import androidx.compose.foundation.gestures.detectTransformGestures
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.material3.Icon
import androidx.compose.material3.IconButton
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Surface
import androidx.compose.runtime.*
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.graphicsLayer
import androidx.compose.ui.input.pointer.pointerInput
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.res.painterResource
import androidx.compose.ui.tooling.preview.Preview
import androidx.compose.ui.unit.dp
import coil.compose.AsyncImage
import coil.request.ImageRequest
import com.android.ssamr.R
import com.android.ssamr.ui.theme.SSAMRTheme
import com.google.accompanist.systemuicontroller.rememberSystemUiController
import kotlin.math.max
import kotlin.math.min

@Composable
fun FullscreenPhotoScreen(
    imageUrl: String,
    onBack: () -> Unit
) {
    val sysUi = rememberSystemUiController()
    LaunchedEffect(Unit) { sysUi.isSystemBarsVisible = false }
    DisposableEffect(Unit) { onDispose { sysUi.isSystemBarsVisible = true } }

    var scale by remember { mutableFloatStateOf(1f) }
    var offset by remember { mutableStateOf(Offset.Zero) }

    fun reset() { scale = 1f; offset = Offset.Zero }

    BackHandler { onBack() }

    Surface(color = Color.Black) {
        Box(Modifier.fillMaxSize()) {
            AsyncImage(
                model = ImageRequest.Builder(LocalContext.current)
                    .data(imageUrl)
                    .crossfade(true)
                    .build(),
                contentDescription = null,
                modifier = Modifier
                    .fillMaxSize()
                    .graphicsLayer {
                        scaleX = scale
                        scaleY = scale
                        translationX = offset.x
                        translationY = offset.y
                    }
                    .pointerInput(Unit) {
                        detectTransformGestures { _, pan, zoom, _ ->
                            val newScale = (scale * zoom).coerceIn(1f, 4f)
                            // 스케일 변화에 맞춰 팬도 반영
                            val scaleChange = newScale / scale
                            scale = newScale
                            offset += pan * scaleChange
                        }
                    }
                    .pointerInput(Unit) {
                        detectTapGestures(
                            onDoubleTap = { reset() }
                        )
                    }
            )

            // 닫기 버튼
            IconButton(
                onClick = onBack,
                modifier = Modifier
                    .align(Alignment.TopStart)
            ) {
                Icon(
                    painter = painterResource(R.drawable.ic_close),
                    contentDescription = "닫기",
                    tint = Color.White
                )
            }
        }
    }
}

@Preview(showBackground = true)
@Composable
fun FullscreenPhotoScreenPreview() {
    SSAMRTheme {
        FullscreenPhotoScreen(imageUrl = "", onBack = {})
    }
}
