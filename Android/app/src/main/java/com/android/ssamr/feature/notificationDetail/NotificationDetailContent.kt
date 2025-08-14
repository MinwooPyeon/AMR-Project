package com.android.ssamr.feature.notificationDetail

import androidx.compose.foundation.background
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.aspectRatio
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.Card
import androidx.compose.material3.CardDefaults
import androidx.compose.material3.Icon
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Surface
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.layout.ContentScale
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.res.painterResource
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.tooling.preview.Preview
import androidx.compose.ui.unit.dp
import coil.compose.AsyncImage
import coil.request.ImageRequest
import com.android.ssamr.BuildConfig
import com.android.ssamr.R
import com.android.ssamr.core.common.time.formatRelativeTimeKorean
import com.android.ssamr.core.domain.model.Notification
import com.android.ssamr.core.domain.model.NotificationAction
import com.android.ssamr.ui.theme.SSAMRTheme

@Composable
fun NotificationDetailInfoCard(
    notification: Notification,
    modifier: Modifier = Modifier
) {
    val (iconRes, accent) = when (notification.riskLevel) {
        NotificationAction.DANGER -> R.drawable.ic_error to Color(0xFFF7575C)
        NotificationAction.WARNING -> R.drawable.ic_warning to Color(0xFFF7B500)
        NotificationAction.INFORMATION -> R.drawable.ic_info to Color(0xFF508DFF)
        else -> R.drawable.ic_info to Color(0xFF9CA3AF)
    }

    Card(
        modifier = modifier
            .fillMaxWidth()
            .padding(16.dp),
        shape = RoundedCornerShape(18.dp),
        colors = CardDefaults.cardColors(containerColor = Color.White),
        elevation = CardDefaults.cardElevation(defaultElevation = 2.dp)
    ) {
        Column(modifier = Modifier.padding(24.dp)) {
            Row(verticalAlignment = Alignment.Top) {
                Box(
                    modifier = Modifier
                        .size(48.dp)
                        .clip(RoundedCornerShape(10.dp))
                        .background(accent.copy(alpha = 0.12f)),
                    contentAlignment = Alignment.Center
                ) {
                    Icon(
                        painter = painterResource(iconRes),
                        contentDescription = null,
                        tint = accent,
                        modifier = Modifier.size(24.dp)
                    )
                }
                Spacer(Modifier.width(12.dp))
                Column(Modifier.weight(1f)) {
                    Row(verticalAlignment = Alignment.CenterVertically) {
                        Text(
                            text = notification.title,
                            style = MaterialTheme.typography.titleLarge,
                            fontWeight = FontWeight.ExtraBold,
                            color = Color(0xFF1F2937)
                        )
                        Spacer(Modifier.width(6.dp))
                        Box(
                            modifier = Modifier
                                .size(8.dp)
                                .background(accent, CircleShape)
                        )
                    }
                    Spacer(Modifier.height(6.dp))
                    Text(
                        text = notification.content,
                        style = MaterialTheme.typography.bodyMedium,
                        color = Color(0xFF4B5563)
                    )
                    Spacer(Modifier.height(14.dp))
                    /* 시간 · 위치 라인 */
                    Row(verticalAlignment = Alignment.CenterVertically) {
                        Icon(
                            painter = painterResource(R.drawable.ic_time), // 작은 시계 벡터
                            contentDescription = null,
                            tint = Color(0xFF9CA3AF),
                            modifier = Modifier.size(16.dp)
                        )
                        Spacer(Modifier.width(6.dp))
                        Text(
                            text = formatRelativeTimeKorean(notification.createAt), // “5분 전” 혹은 포맷된 시간
                            style = MaterialTheme.typography.bodySmall,
                            color = Color(0xFF6B7280)
                        )
                        Spacer(Modifier.width(14.dp))
                        Icon(
                            painter = painterResource(R.drawable.current_location),
                            contentDescription = null,
                            tint = Color(0xFF9CA3AF),
                            modifier = Modifier.size(16.dp)
                        )
                        Spacer(Modifier.width(6.dp))
                        Text(
                            text = notification.area,
                            style = MaterialTheme.typography.bodySmall,
                            color = Color(0xFF6B7280)
                        )
                    }
                }
            }
        }
    }
}

@Composable
fun CurrentSituationPhoto(
    imageUrl: String?,
    timeText: String?,               // 예: "오후 1:31:57"
    modifier: Modifier = Modifier,
    onClick: (() -> Unit)? = null
) {

    Surface(
        shape = RoundedCornerShape(24.dp),
        color = Color.White,
        shadowElevation = 2.dp,
        modifier = modifier.padding(16.dp)
    ){
        Column(
            modifier = modifier.fillMaxWidth().padding(24.dp)
        ) {
            Text(
                text = "Current Situation",
                style = MaterialTheme.typography.titleLarge.copy(fontWeight = FontWeight.Bold),
                color = Color(0xFF374151)
            )
            Spacer(Modifier.height(12.dp))

            Card(
                shape = RoundedCornerShape(18.dp),
                colors = CardDefaults.cardColors(containerColor = Color.White),
                elevation = CardDefaults.cardElevation(defaultElevation = 2.dp),
                onClick = { onClick?.invoke() }
            ) {
                Box(
                    modifier = Modifier
                        .fillMaxWidth()
                        .aspectRatio(16f / 9f)
                        .clip(RoundedCornerShape(18.dp))
                        .background(Color(0xFFE5E7EB))
                ) {
                    AsyncImage(
                        model = ImageRequest.Builder(LocalContext.current)
                            .data("${BuildConfig.BASE_URL.dropLast(1)}${imageUrl}")
                            .crossfade(true)
                            .placeholder(R.drawable.ic_image_placeholder) // 선택
                            .error(R.drawable.ic_image_placeholder)       // 선택
                            .build(),
                        contentDescription = null,
                        contentScale = ContentScale.Fit,
                        modifier = Modifier.fillMaxSize()
                    )

                    if (!timeText.isNullOrBlank()) {
                        Text(
                            text = timeText,
                            style = MaterialTheme.typography.labelMedium,
                            color = Color.White,
                            modifier = Modifier
                                .align(Alignment.TopEnd)
                                .padding(10.dp)
                                .clip(RoundedCornerShape(6.dp))
                                .background(Color(0x99000000))
                                .padding(horizontal = 8.dp, vertical = 4.dp)
                        )
                    }
                }
            }
        }
    }

}

@Preview(showBackground = true)
@Composable
fun NotificationDetailInfoCardPreview() {
    val sampleNotification = Notification(
        id = 1L,
        title = "알림 제목 1",
        content = "알림 내용 1",
        riskLevel = NotificationAction.DANGER,
        case = "화재",
        area = "A구역-1",
        createAt = "3분전",
        image = "",
        serial = "AMR001"
    )

    SSAMRTheme {
        NotificationDetailInfoCard(
            notification = sampleNotification
        )
    }
}

@Preview(showBackground = true)
@Composable
fun AlertImageSectionPreview() {
    SSAMRTheme {
        CurrentSituationPhoto(
            imageUrl = "",
            timeText = "오후 12:30:00", // 포맷해서 넘기면 더 예쁨
            onClick = { /* 전체보기 등 */ }
        )
    }
}
