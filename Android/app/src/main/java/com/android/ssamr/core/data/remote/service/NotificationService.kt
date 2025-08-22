package com.android.ssamr.core.data.remote.service

import com.android.ssamr.core.data.model.amr.response.NotificationDto
import com.android.ssamr.core.data.model.notification.request.MarkReadRequest
import retrofit2.http.Body
import retrofit2.http.GET
import retrofit2.http.PATCH
import retrofit2.http.POST
import retrofit2.http.Path

interface NotificationService {
    @GET("notifications")
    suspend fun getNotifications(): List<NotificationDto>

    @PATCH("notifications/{id}")
    suspend fun markRead(@Path("id") id: Long, @Body request: MarkReadRequest)

    @GET("notifications/{id}")
    suspend fun getNotificationDetail(@Path("id") id: Long): NotificationDto
}