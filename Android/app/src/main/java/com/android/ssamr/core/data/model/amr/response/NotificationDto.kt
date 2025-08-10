package com.android.ssamr.core.data.model.amr.response

data class NotificationDto (
    val id: Long,
    val title: String,
    val content: String,
    val riskLevel: String,
    val date: String,
    val image: String?,
    val location: String,
    val isRead: Boolean = false,
    val readAt: Long? = null
)