package com.android.ssamr.core.network.api

import retrofit2.Response
import retrofit2.http.POST
import retrofit2.http.Path

interface AmrControlApi {
    @POST("amr/{id}/manual-start")
    suspend fun requestManualStart(@Path("id") id: Long): Response<Unit>

    @POST("amr/{id}/manual-return")
    suspend fun requestManualReturn(@Path("id") id: Long): Response<Unit>
}