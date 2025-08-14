package com.android.ssamr.core.data.remote.service

import retrofit2.http.Body
import retrofit2.http.POST

interface FcmService {

    @POST("fcm")
    suspend fun registerFcmToken(@Body token: String)
}