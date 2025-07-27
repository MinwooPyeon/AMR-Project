package com.android.ssamr.core.ui

import androidx.compose.animation.core.RepeatMode
import androidx.compose.animation.core.animateFloat
import androidx.compose.animation.core.infiniteRepeatable
import androidx.compose.animation.core.keyframes
import androidx.compose.animation.core.rememberInfiniteTransition
import androidx.compose.foundation.background
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.tooling.preview.Preview
import androidx.compose.ui.unit.dp
import com.android.ssamr.ui.theme.SSAMRTheme

@Composable
fun DotsLoading(
    dotCount: Int = 3,
    dotSize: Int = 10,
    dotColor: Color = Color(0xFF3B82F6),
    modifier: Modifier = Modifier
) {
    val infiniteTransition = rememberInfiniteTransition(label = "dots")
    val alphas = List(dotCount) { i ->
        infiniteTransition.animateFloat(
            initialValue = 0.2f,
            targetValue = 1f,
            animationSpec = infiniteRepeatable(
                animation = keyframes {
                    durationMillis = 1200
                    0.2f at i * 200
                    1f at (i * 200 + 400)
                },
                repeatMode = RepeatMode.Restart
            ),
            label = "dot$i"
        )
    }

    Row(modifier = modifier, verticalAlignment = Alignment.CenterVertically) {
        repeat(dotCount) { i ->
            Box(
                Modifier
                    .size(dotSize.dp)
                    .background(dotColor.copy(alpha = alphas[i].value), CircleShape)
            )
            if (i < dotCount - 1) Spacer(Modifier.width(8.dp))
        }
    }
}

@Preview(showBackground = true)
@Composable
fun DotsLoadingPreview() {
    SSAMRTheme {
        DotsLoading()
    }
}
