package com.android.ssamr.core.data.local.entity

import androidx.room.Entity
import androidx.room.PrimaryKey

@Entity(tableName = "notifications")
data class NotificationEntity(
    @PrimaryKey(autoGenerate = true) val id: Long = 0,
    val title: String,
    val content: String,
    val riskLevel: String,
    val date: String,
    val image: String?,
    val location: String,
    val isRead: Boolean,
    val readAt: Long? = null
)