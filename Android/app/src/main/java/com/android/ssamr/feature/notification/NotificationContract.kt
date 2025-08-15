package com.android.ssamr.feature.notification

import com.android.ssamr.core.domain.model.Notification
import com.android.ssamr.core.domain.model.NotificationCategory

data class NotificationState (
    val selectedCategory: NotificationCategory = NotificationCategory.ALL,
    val fullNotificationList: List<Notification> = emptyList(),
    val notificationList: List<Notification> = emptyList(),
    val categoryCounts: Map<NotificationCategory, Int> = emptyMap(),
    val isCallDialogVisible: Boolean = false,
    val isLoading: Boolean = false,
    val error: String? = null
)

sealed class NotificationIntent {
    data class ClickNotificationCategory(val category: NotificationCategory) : NotificationIntent()

    data class ClickNotificationCard(val notificationId: Long) : NotificationIntent()

    data object OpenCallDialog : NotificationIntent()
    data object DismissCallDialog : NotificationIntent()
    data object CallManager : NotificationIntent()
    data object CallFire : NotificationIntent()
}

sealed class NotificationEffect{
    data class NavigateToNotificationDetail(val notificationId: Long) : NotificationEffect()
    data class ShowError(val message: String) : NotificationEffect()
    data class LaunchDialer(val phone: String) : NotificationEffect()

}