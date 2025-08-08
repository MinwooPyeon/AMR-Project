package com.android.ssamr.feature.dashboard.fullscreenmap

import androidx.compose.foundation.Image
import androidx.compose.foundation.background
import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.*
import androidx.compose.material3.*
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.painter.BitmapPainter
import androidx.compose.ui.layout.ContentScale
import androidx.compose.ui.platform.LocalDensity
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import androidx.compose.ui.tooling.preview.Preview


import androidx.compose.ui.graphics.ImageBitmap
import androidx.compose.ui.graphics.asImageBitmap
import android.graphics.Bitmap
import android.graphics.Canvas
import android.graphics.Color as AndroidColor
import android.graphics.Paint
import com.android.ssamr.core.domain.model.AmrMapPosition
import com.android.ssamr.core.domain.model.DashboardAmrStatus

@Composable
fun FullscreenMapContent(
    state: FullscreenMapState,
    sendIntent: (FullscreenMapIntent) -> Unit
) {
    Box(modifier = Modifier.fillMaxSize()) {
        if (state.mapImage != null) {
            MapImageBackground(state)
            AmrMarkers(state, sendIntent)
        }

        else {
            EmptyMapFallback()
        }

        if (state.isLoading) {
            LoadingOverlay()
        }
    }
}

@Composable
private fun MapImageBackground(state: FullscreenMapState) {
    Image(
        painter = BitmapPainter(state.mapImage!!),
        contentDescription = "공장 지도",
        modifier = Modifier.fillMaxSize(),
        contentScale = ContentScale.Crop
    )
}

@Composable
private fun AmrMarkers(
    state: FullscreenMapState,
    sendIntent: (FullscreenMapIntent) -> Unit
) {
    state.amrPositions.forEach { amr ->
        val markerSize = 24.dp
        val offsetX = with(LocalDensity.current) { amr.x.toDp() }
        val offsetY = with(LocalDensity.current) { amr.y.toDp() }

        Box(
            modifier = Modifier
                .size(markerSize)
                .offset(x = offsetX - markerSize / 2, y = offsetY - markerSize / 2)
                .background(
                    color = amr.status.color.copy(alpha = 0.8f),
                    shape = MaterialTheme.shapes.small
                )
                .clickable { sendIntent(FullscreenMapIntent.ClickAmr(amr.id)) },
            contentAlignment = Alignment.Center
        ) {
            Text(
                text = amr.name.split("-").last(),
                fontSize = 10.sp,
                fontWeight = FontWeight.Bold,
                color = Color.White
            )
        }
    }
}

@Composable
private fun EmptyMapFallback() {
    Box(
        modifier = Modifier
            .fillMaxSize()
            .background(Color.LightGray),
        contentAlignment = Alignment.Center
    ) {
        Text("지도를 불러오는 중...", color = Color.DarkGray)
    }
}



@Composable
private fun LoadingOverlay() {
    Box(
        modifier = Modifier
            .fillMaxSize()
            .background(Color.Black.copy(alpha = 0.4f)),
        contentAlignment = Alignment.Center
    ) {
        CircularProgressIndicator(color = Color.White)
    }
}

fun generateDummyMapImage(width: Int = 360, height: Int = 640): ImageBitmap {
    val bitmap = Bitmap.createBitmap(width, height, Bitmap.Config.ARGB_8888)
    val canvas = Canvas(bitmap)
    val paint = Paint()
    paint.color = AndroidColor.LTGRAY
    canvas.drawRect(0f, 0f, width.toFloat(), height.toFloat(), paint)
    return bitmap.asImageBitmap()
}

@Preview(showBackground = true, widthDp = 360, heightDp = 640)
@Composable
fun FullscreenMapContentPreview() {
    val previewState = FullscreenMapState(
        isLoading = false,
        amrPositions = listOf(
            AmrMapPosition(1L, "AMR-001", x = 100f, y = 150f, status = DashboardAmrStatus.RUNNING),
            AmrMapPosition(2L, "AMR-002", x = 200f, y = 300f, status = DashboardAmrStatus.CHARGING),
            AmrMapPosition(3L, "AMR-003", x = 120f, y = 400f, status = DashboardAmrStatus.CHECKING),
        ),
        mapImage = generateDummyMapImage()
    )

    MaterialTheme {
        FullscreenMapContent(
            state = previewState,
            sendIntent = {}
        )
    }
}

