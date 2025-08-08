package com.android.ssamr.core.data.remote.service

import com.android.ssamr.core.network.model.ApiResponse
import retrofit2.http.GET

interface DashboardService {
    @GET("api/v1/dashboard/map/image")
    suspend fun getMapImage(): ApiResponse<String> // base64 or URL
}