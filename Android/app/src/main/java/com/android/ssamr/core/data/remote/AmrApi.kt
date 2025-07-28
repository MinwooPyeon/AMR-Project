package com.android.ssamr.core.data.remote

import retrofit2.Response
import retrofit2.http.Body
import retrofit2.http.POST

interface AmrApi {
    @POST("api/amr/manual-start")
    suspend fun manualStart(@Body request: ManualCommandRequest): Response<Unit>

    @POST("api/amr/manual-return")
    suspend fun manualReturn(@Body request: ManualCommandRequest): Response<Unit>
}

data class ManualCommandRequest(val amrId: Long)
