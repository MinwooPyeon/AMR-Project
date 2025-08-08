package com.android.ssamr.feature.notification

import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.height
import androidx.compose.runtime.Composable
import androidx.compose.ui.Modifier
import androidx.compose.ui.tooling.preview.Preview
import androidx.compose.ui.unit.dp
import com.android.ssamr.core.domain.model.Notification
import com.android.ssamr.core.domain.model.NotificationAction
import com.android.ssamr.core.domain.model.NotificationCategory

@Composable
fun NotificationScreen(
    state: NotificationState,
    sendIntent: (NotificationIntent) -> Unit = {}
) {
    Box(
        modifier = Modifier
            .fillMaxSize()
    ) {
        Column(modifier = Modifier.fillMaxSize()) {
            NotificationCategoryTabRow(state = state, sendIntent = sendIntent)

            Spacer(Modifier.height(8.dp))

            NotificationList(
                notification = state.notificationList,
                onNotificationCardClick = { notificationId ->
                    sendIntent(NotificationIntent.ClickNotificationCard(notificationId))
                }
            )


        }
    }
}

@Preview(showBackground = true)
@Composable
fun NotificationScreenPreview() {
    NotificationScreen(
        state = NotificationState(
            selectedCategory = NotificationCategory.ALL,
            notificationList = sampleNotifications,
            categoryCounts = mapOf(
                NotificationCategory.ALL to 5,
                NotificationCategory.DANGER to 2,
                NotificationCategory.WARNING to 2,
                NotificationCategory.INFORMATION to 1
            ),
            isLoading = false,
            error = null
        ),
        sendIntent = {}
    )
}

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
        content = "알림 내용 1 알림 내용 1 알림 내용 1 알림 내용 1 알림 내용 1 알림 내용 1 알림 내용 1 알림 내용 1 알림 내용 1 알림 내용 1 알림 내용 1 알림 내용 1 알림 내용 1 알림 내용 1 알림 내용 1 알림 내용 1 알림 내용 1 알림 내용 1 알림 내용 1 ",
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