package com.android.ssamr.core.data.repository

import com.android.ssamr.core.data.model.amr.response.DashMapDto
import com.android.ssamr.core.data.model.amr.response.DashboardDto
import com.android.ssamr.core.data.remote.datasource.DashboardRemoteDataSource
import com.android.ssamr.core.domain.repository.DashboardRepository
import javax.inject.Inject

class DashboardRepositoryImpl @Inject constructor(
    private val remoteDataSource: DashboardRemoteDataSource
) : DashboardRepository {

    override suspend fun getDashboardAmrs(): List<DashboardDto> {
        return remoteDataSource.getDashboardAmrs()
    }

    override suspend fun getMapAmrs(): List<DashMapDto> {
        return remoteDataSource.getDashMapPositions()
    }
}