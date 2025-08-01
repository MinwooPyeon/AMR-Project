package com.android.ssamr.core.data.remote.datasource

import com.android.ssamr.core.data.model.amr.response.DashMapDto
import com.android.ssamr.core.data.model.amr.response.DashboardDto
import com.android.ssamr.core.data.remote.service.DashboardService
import javax.inject.Inject

class DashboardRemoteDataSource @Inject constructor(
    private val service: DashboardService
){
    suspend fun getDashboardAmrs(): List<DashboardDto> {
        val response = service.getDashboardAmrs()
        return response.data ?: emptyList()
    }

    suspend fun getDashMapPositions(): List<DashMapDto> {
        val response = service.getMapAmrs()
        return response.data ?: emptyList()
    }
}