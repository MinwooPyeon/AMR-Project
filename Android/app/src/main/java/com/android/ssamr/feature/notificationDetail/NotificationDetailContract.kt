package com.android.ssamr.feature.notificationDetail

import com.android.ssamr.core.domain.model.Notification

data class NotificationDetailState(
    val id: Long = 0L,
    val data: Notification? = null,
    val isLoading: Boolean = true,
    val error: String? = null
)

sealed class NotificationDetailIntent {
    data object ClickMarkRead : NotificationDetailIntent()
    data class ClickPhotoView(val url: String?) : NotificationDetailIntent()
}

sealed class NotificationDetailEffect {
    data class NavigateToPhotoView(val url: String?) : NotificationDetailEffect()
    data class ShowError(val message: String) : NotificationDetailEffect()
}
