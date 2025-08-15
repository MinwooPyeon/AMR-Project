package com.android.ssamr.core.data.remote.service

import com.android.ssamr.core.data.model.fcm.request.FcmTokenRequest
import retrofit2.Response
import retrofit2.http.Body
import retrofit2.http.POST

interface FcmService {

    @POST("fcm")
    suspend fun registerFcmToken(@Body request: FcmTokenRequest): Response<Unit>
}