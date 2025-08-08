package com.android.ssamr.feature.notification

import androidx.compose.foundation.background
import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.IntrinsicSize
import androidx.compose.foundation.layout.PaddingValues
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxHeight
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.layout.widthIn
import androidx.compose.foundation.lazy.LazyColumn
import androidx.compose.foundation.lazy.LazyRow
import androidx.compose.foundation.lazy.items
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.Card
import androidx.compose.material3.CardDefaults
import androidx.compose.material3.Icon
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Text
import androidx.compose.material3.TextButton
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.vector.ImageVector
import androidx.compose.ui.res.painterResource
import androidx.compose.ui.res.vectorResource
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.style.TextOverflow
import androidx.compose.ui.tooling.preview.Preview
import androidx.compose.ui.unit.dp
import com.android.ssamr.R
import com.android.ssamr.core.domain.model.AmrAction
import com.android.ssamr.core.domain.model.AmrCategory
import com.android.ssamr.core.domain.model.AmrStatus
import com.android.ssamr.core.domain.model.Notification
import com.android.ssamr.core.domain.model.NotificationAction
import com.android.ssamr.core.domain.model.NotificationCategory
import com.android.ssamr.feature.amr.AmrCard
import com.android.ssamr.feature.amr.AmrIntent
import com.android.ssamr.feature.amr.AmrState
import com.android.ssamr.ui.theme.SSAMRTheme

@Composable
fun NotificationList(
    notification: List<Notification>,
    onNotificationCardClick: (Long) -> Unit
) {
    LazyColumn(
        modifier = Modifier
            .padding(horizontal = 8.dp)
    ) {
        items(notification) { noti ->
            NotificationCard(notification = noti, onNotificationCardClick)
        }
    }
}

@Composable
fun NotificationCategoryTabRow(
    state: NotificationState,
    sendIntent: (NotificationIntent) -> Unit
) {
    val categories = NotificationCategory.values()
    Box(
        Modifier
            .fillMaxWidth()
            .padding(vertical = 8.dp)
    ) {
        LazyRow(
            modifier = Modifier.fillMaxWidth(),
            verticalAlignment = Alignment.CenterVertically,
            contentPadding = PaddingValues(horizontal = 12.dp),
            horizontalArrangement = Arrangement.spacedBy(8.dp)
        ) {
            items(categories) { category ->
                val selected = category == state.selectedCategory
                TextButton(
                    onClick = { sendIntent(NotificationIntent.ClickNotificationCategory(category)) },
                    modifier = Modifier
                        .height(60.dp)
                        .widthIn(min = 80.dp)
                ) {
                    Text(
                        text = "${category.label} (${state.categoryCounts[category] ?: 0})",
                        color = if (selected) Color.White else Color(0xFF828282),
                        maxLines = 1,
                        overflow = TextOverflow.Ellipsis,
                        modifier = Modifier
                            .background(
                                if (selected) Color(0xFF4C65E2) else Color(0xFFF3F4F6),
                                shape = RoundedCornerShape(20.dp)
                            )
                            .padding(horizontal = 20.dp, vertical = 12.dp)
                    )
                }
            }
        }
    }
}

@Composable
fun NotificationCard(
    notification: Notification,
    onNotificationCardClick: (Long) -> Unit
) {
    // 타입별 색상/아이콘
    val (iconRes, baseStatusColor) = when (notification.riskLevel) {
        NotificationAction.DANGER -> R.drawable.ic_error to Color(0xFFF7575C)
        NotificationAction.WARNING -> R.drawable.ic_warning to Color(0xFFF7B500)
        NotificationAction.INFORMATION -> R.drawable.ic_info to Color(0xFF508DFF)
        else -> R.drawable.ic_info to Color.Gray
    }

    // 읽음 스타일
    val containerColor =
        if (notification.isRead) MaterialTheme.colorScheme.surfaceVariant
        else Color.White
    val statusColor =
        if (notification.isRead) baseStatusColor.copy(alpha = 0.45f)
        else baseStatusColor
    val titleColor =
        if (notification.isRead) MaterialTheme.colorScheme.onSurface.copy(alpha = 0.8f)
        else Color(0xFF191F28)
    val bodyColor =
        if (notification.isRead) MaterialTheme.colorScheme.onSurface.copy(alpha = 0.7f)
        else Color(0xFF404150)
    val metaColor =
        if (notification.isRead) MaterialTheme.colorScheme.onSurfaceVariant
        else Color(0xFF8C8D96)
    val elevation =
        if (notification.isRead) 0.dp else 2.dp

    Card(
        modifier = Modifier
            .fillMaxWidth()
            .padding(horizontal = 12.dp, vertical = 6.dp)
            .clickable { onNotificationCardClick(notification.id) },
        shape = RoundedCornerShape(16.dp),
        colors = CardDefaults.cardColors(containerColor = containerColor),
        elevation = CardDefaults.cardElevation(defaultElevation = elevation)
    ) {
        Column(
            modifier = Modifier
                .fillMaxWidth()
                .padding(vertical = 16.dp, horizontal = 16.dp)
        ) {
            Row(verticalAlignment = Alignment.CenterVertically) {

                // 타입 아이콘 박스 (읽음이면 더 옅게)
                Box(
                    modifier = Modifier
                        .size(44.dp)
                        .background(statusColor.copy(alpha = 0.12f), RoundedCornerShape(8.dp)),
                    contentAlignment = Alignment.Center
                ) {
                    Icon(
                        painter = painterResource(id = iconRes),
                        contentDescription = null,
                        tint = statusColor,
                        modifier = Modifier.size(24.dp)
                    )
                }

                Spacer(Modifier.width(12.dp))

                Column(modifier = Modifier.weight(1f)) {
                    Row(verticalAlignment = Alignment.CenterVertically) {
                        Text(
                            text = notification.title,
                            style = MaterialTheme.typography.titleMedium,
                            fontWeight = FontWeight.Bold,
                            color = titleColor
                        )
                        Spacer(Modifier.width(6.dp))

                        // 상태 점
                        Box(
                            modifier = Modifier
                                .size(8.dp)
                                .background(statusColor, CircleShape)
                        )

                        Spacer(Modifier.weight(1f))

                        // 시간
                        Text(
                            text = notification.date,
                            style = MaterialTheme.typography.labelSmall,
                            color = metaColor
                        )
                    }

                    Spacer(Modifier.height(4.dp))

                    Text(
                        text = notification.content,
                        style = MaterialTheme.typography.bodyMedium,
                        color = bodyColor
                    )

                    Spacer(Modifier.height(10.dp))

                    Row(verticalAlignment = Alignment.CenterVertically) {
                        Icon(
                            painter = painterResource(id = R.drawable.current_location),
                            contentDescription = null,
                            tint = metaColor.copy(alpha = 0.9f),
                            modifier = Modifier.size(16.dp)
                        )
                        Spacer(Modifier.width(4.dp))
                        Text(
                            text = notification.location,
                            style = MaterialTheme.typography.bodySmall,
                            color = metaColor
                        )
                    }
                }
            }
        }
    }
}


@Preview(showBackground = true)
@Composable
fun NotificationCategoryTabRowPreview() {
    SSAMRTheme {
        NotificationCategoryTabRow(
            NotificationState()
        ) {}

    }
}

@Preview(showBackground = true)
@Composable
fun NotificationListPreview() {
    SSAMRTheme {
        val sampleNotifications = listOf(
            Notification(
                id = 1L,
                title = "알림 제목 1",
                content = "알림 내용 1",
                riskLevel = NotificationAction.DANGER,
                location = "A구역-1",
                date = "3분전",
                image = "",
                isRead = false
                ),
            Notification(
                id = 1L,
                title = "알림 제목 1",
                content = "알림 내용 1",
                riskLevel = NotificationAction.WARNING,
                location = "A구역-1",
                date = "3분전",
                image = "",
                isRead = true
            ),
            Notification(
                id = 1L,
                title = "알림 제목 1",
                content = "알림 내용 1",
                riskLevel = NotificationAction.INFORMATION,
                location = "A구역-1",
                date = "3분전",
                image = "",
                isRead = true
            ),
            Notification(
                id = 1L,
                title = "알림 제목 1",
                content = "알림 내용 1",
                riskLevel = NotificationAction.DANGER,
                location = "A구역-1",
                date = "3분전",
                image = "",
                isRead = false
            ),
            Notification(
                id = 1L,
                title = "알림 제목 1",
                content = "알림 내용 1",
                riskLevel = NotificationAction.DANGER,
                location = "A구역-1",
                date = "3분전",
                image = "",
                isRead = true
            )
        )

        SSAMRTheme {
            Column(Modifier.padding(16.dp)) {
                NotificationList(notification = sampleNotifications) { }
            }
        }
    }

}
