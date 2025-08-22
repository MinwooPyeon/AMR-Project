package com.android.ssamr.core.data.remote.service

import com.android.ssamr.core.network.model.ApiResponse
import retrofit2.http.GET

interface DashboardService {
    @GET("map/image")
    suspend fun getMapImage(): ApiResponse<String> // base64 or URL
}

// http://192.168.100.141:8080/images/map.png