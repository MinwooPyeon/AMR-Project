package com.android.ssamr.feature.amrWebcam

import android.content.Context
import android.util.Log
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.Surface
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.DisposableEffect
import androidx.compose.runtime.remember
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.tooling.preview.Preview
import androidx.compose.ui.unit.dp
import androidx.compose.ui.viewinterop.AndroidView
import androidx.media3.common.MediaItem
import androidx.media3.exoplayer.ExoPlayer
import androidx.media3.ui.PlayerView
import com.android.ssamr.core.ui.LiveRedDot
import com.android.ssamr.ui.theme.SSAMRTheme

@Composable
fun AmrWebcamInfoPanel (
    state: AmrWebcamState,
    modifier: Modifier = Modifier
) {
    Surface(
        color = Color.Black.copy(alpha = 0.6f),
        shape = RoundedCornerShape(16.dp),
        modifier = modifier
    ) {
        Column(modifier = Modifier.padding(16.dp)) {
            Row(
                Modifier.fillMaxWidth(),
                horizontalArrangement = Arrangement.SpaceBetween
            ) {
                Row(verticalAlignment = Alignment.CenterVertically) {
                    LiveRedDot(modifier = Modifier)
                    Spacer(Modifier.width(8.dp))
                    Text("실시간", color = Color.White)
                }
                Text(state.lastUpdated, color = Color.White)
//                Text("오후 5:30:12", color = Color.White)
            }
            Spacer(Modifier.height(8.dp))
            Row(
                Modifier.fillMaxWidth(),
                horizontalArrangement = Arrangement.SpaceBetween
            ) {
                Text("위치\n${state.amr?.location}", color = Color.White)
                Text("상태\n${state.amr?.status}", color = Color.White)
//                Text("위치\nA구역-라인1", color = Color.White)
//                Text("상태\n작동중", color = Color.White)
                Spacer(Modifier.width(16.dp))
            }
        }
    }
}

@Composable
fun RtspPlayerView(
    url: String,
    modifier: Modifier = Modifier
) {
    Log.d("RTSP", "RtspPlayerView 실행, url=$url")

    if (url.isNullOrBlank()) {
        // url이 비어있으면 ExoPlayer 생성하지 않음
        Log.d("RTSP", "RtspPlayerView: 빈 url로 인해 생성하지 않음")
        return
    }

    val context = LocalContext.current

    // ExoPlayer 인스턴스 생성 및 세팅
    val exoPlayer = remember {
        ExoPlayer.Builder(context).build().apply {
            setMediaItem(MediaItem.fromUri(url))
            prepare()
            playWhenReady = true
        }
    }

    // PlayerView를 Compose에 임베딩
    AndroidView(
        modifier = modifier,
        factory = { ctx: Context ->
            PlayerView(ctx).apply {
                player = exoPlayer
                useController = true // 재생/일시정지 등 컨트롤러 표시 여부
            }
        }
    )
    Log.d("RTSP", "RtspPlayerView: $url")

    // 컴포저블이 dispose될 때 플레이어도 정리
    DisposableEffect(Unit) {
        onDispose {
            exoPlayer.release()
        }
    }
}

@Preview(showBackground = true)
@Composable
fun AmrWebcamInfoPanelPreview() {
    SSAMRTheme {
        AmrWebcamInfoPanel(
            state = AmrWebcamState()
        )
    }
}

@Preview(showBackground = true)
@Composable
fun RtspPlayerViewPreview() {
    SSAMRTheme {
        RtspPlayerView(
            url = "rtsp://192.168.0.123:554/live.sdp",
            modifier = Modifier.fillMaxSize()
        )
    }
}