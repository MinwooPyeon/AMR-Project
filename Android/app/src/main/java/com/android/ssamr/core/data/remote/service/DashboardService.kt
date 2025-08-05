package com.android.ssamr.core.data.remote.service

import com.android.ssamr.core.data.model.amr.response.DashMapDto
import com.android.ssamr.core.data.model.amr.response.DashboardDto
import com.android.ssamr.core.network.model.ApiResponse
import retrofit2.http.GET

interface DashboardService {
    @GET("api/v1/dashboard/list")
    suspend fun getDashboardAmrs(): ApiResponse<List<DashboardDto>>

    @GET("api/v1/dashboard/map/positions")
    suspend fun getMapAmrPositions(): ApiResponse<List<DashMapDto>>

    @GET("api/v1/dashboard/map/image")
    suspend fun getMapImage(): ApiResponse<String> // base64 or URL
}