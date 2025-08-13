package com.android.ssamr.core.fcm

import com.android.ssamr.core.domain.model.Notification
import com.android.ssamr.core.domain.model.NotificationAction
import javax.inject.Inject

class NotificationRealtimeMapper @Inject constructor() {

    fun fromFcmData(data: Map<String, String>): Notification {
        val id = data["id"]?.toLongOrNull() ?: 0L
        val title = data["title"].orEmpty()
        val content = data["content"].orEmpty()
        val risk = data["riskLevel"]?.uppercase().orEmpty()
        val case = data["case"].orEmpty()
        val date = data["date"].orEmpty()
        val image = data["image"]
        val location = data["location"].orEmpty()
        val serial = data["serial"].orEmpty()

        val riskLevel = when (risk) {
            "DANGER" -> NotificationAction.DANGER
            "WARNING" -> NotificationAction.WARNING
            "INFORMATION" -> NotificationAction.INFORMATION
            else -> NotificationAction.INFORMATION
        }

        return Notification(
            id = id,
            title = title,
            content = content,
            riskLevel = riskLevel,
            case = case,
            createAt = date,
            image = image,
            area = location,
            isRead = false,
            serial = serial
        )
    }
}
