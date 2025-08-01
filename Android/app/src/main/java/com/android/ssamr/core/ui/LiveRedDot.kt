package com.android.ssamr.core.ui

import androidx.compose.animation.core.LinearEasing
import androidx.compose.animation.core.RepeatMode
import androidx.compose.animation.core.animateFloat
import androidx.compose.animation.core.infiniteRepeatable
import androidx.compose.animation.core.rememberInfiniteTransition
import androidx.compose.animation.core.tween
import androidx.compose.foundation.background
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.graphicsLayer
import androidx.compose.ui.tooling.preview.Preview
import androidx.compose.ui.unit.Dp
import androidx.compose.ui.unit.dp
import com.android.ssamr.ui.theme.SSAMRTheme

@Composable
fun LiveRedDot(
    modifier: Modifier = Modifier,
    dotSize: Dp = 12.dp
) {
    // 애니메이션 효과: alpha 값이 0.4 ~ 1.0으로 무한히 깜빡임
    val infiniteTransition = rememberInfiniteTransition(label = "live-dot")
    val alpha by infiniteTransition.animateFloat(
        initialValue = 0.4f,
        targetValue = 1.0f,
        animationSpec = infiniteRepeatable(
            animation = tween(600, easing = LinearEasing),
            repeatMode = RepeatMode.Reverse
        ),
        label = "alpha"
    )
    Box(
        modifier = modifier
            .size(dotSize)
            .graphicsLayer { this.alpha = alpha }
            .background(Color.Red, CircleShape)
    )
}

@Preview(showBackground = true)
@Composable
fun LiveRedDotPreview() {
    SSAMRTheme {
        LiveRedDot()
    }
}
