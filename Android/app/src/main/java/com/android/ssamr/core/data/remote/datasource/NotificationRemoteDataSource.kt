package com.android.ssamr.core.data.remote.datasource

import com.android.ssamr.core.data.model.amr.response.NotificationDto
import com.android.ssamr.core.data.model.notification.request.MarkReadRequest
import com.android.ssamr.core.data.remote.service.NotificationService
import javax.inject.Inject

class NotificationRemoteDataSource @Inject constructor(
    private val service: NotificationService
) {

    suspend fun fetchNotifications(): List<NotificationDto> =
        service.getNotifications()

    suspend fun markRead(id: Long, isRead: Boolean) =
        service.markRead(id, MarkReadRequest(isRead))

    suspend fun getNotificationDetail(id: Long): NotificationDto =
        service.getNotificationDetail(id)
}