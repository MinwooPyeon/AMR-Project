package com.android.ssamr.core.domain.repository

import com.android.ssamr.core.data.model.amr.response.DashboardDto
import com.android.ssamr.core.data.model.amr.response.DashMapDto

interface DashboardRepository {
    suspend fun getDashboardAmrs(): List<DashboardDto>
    suspend fun getMapAmrs(): List<DashMapDto>
}