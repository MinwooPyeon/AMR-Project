package com.android.ssamr.core.data.remote.service

import com.android.ssamr.core.data.model.amr.response.DashMapDto
import com.android.ssamr.core.data.model.amr.response.DashboardDto
import com.android.ssamr.core.network.model.ApiResponse
import retrofit2.http.GET

interface DashboardService {
    @GET("/dashboard/list")
    suspend fun getDashboardAmrs(): ApiResponse<List<DashboardDto>>

    @GET("/dashboard/map/positions")
    suspend fun getMapAmrPositions(): ApiResponse<List<DashMapDto>>

    @GET("/dashboard/map/image")
    suspend fun getMapImage(): ApiResponse<String> // base64 or URL
}