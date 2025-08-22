package com.android.ssamr.core.data.remote.datasource

import com.android.ssamr.core.data.remote.service.DashboardService
import javax.inject.Inject

class DashboardRemoteDataSource @Inject constructor(
    private val service: DashboardService
){
    suspend fun getMapImage(): String {
        val response = service.getMapImage()
        return response.data ?: ""
    }
}