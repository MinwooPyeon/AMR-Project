package com.android.ssamr.core.domain.repository

interface DashboardRepository {
    suspend fun getMapImage(): String
}