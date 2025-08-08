package com.android.ssamr.core.domain.model

data class Notification(
    val id: Long,
    val title: String,
    val content: String,
    val riskLevel: NotificationAction,
    val date: String,
    val image: String?,
    val location: String,
    val isRead: Boolean = false
)


enum class NotificationCategory(val label: String) {
    ALL("전체"),
    DANGER("위험"),
    WARNING("경고"),
    INFORMATION("정보")
}

enum class NotificationAction(val display: String) {
    DANGER("위험"),
    WARNING("경고"),
    INFORMATION("정보")
}
