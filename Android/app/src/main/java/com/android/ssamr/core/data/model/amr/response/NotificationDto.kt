package com.android.ssamr.core.data.model.amr.response

data class NotificationDto (
    val id: Long,
    val title: String,
    val content: String,
    val area: String,
    val case: String,
    val image: String?,
    val isRead: Boolean = false,
    val readAt: Long? = null,
    val createAt: String,
)