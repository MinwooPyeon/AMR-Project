package com.android.ssamr.core.data.remote.datasource

import com.android.ssamr.core.data.model.amr.response.NotificationDto
import com.android.ssamr.core.data.remote.service.NotificationService
import javax.inject.Inject

class NotificationRemoteDataSource @Inject constructor(
    private val service: NotificationService
) {

    suspend fun fetchNotifications(): List<NotificationDto> =
        service.getNotifications()

    suspend fun markRead(id: Long) =
        service.markRead(id)
}