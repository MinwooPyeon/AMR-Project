package com.android.ssamr.core.data.remote.service

import com.android.ssamr.core.data.model.amr.response.AmrDetailDto
import com.android.ssamr.core.data.model.amr.response.AmrDto
import com.android.ssamr.core.network.model.ApiResponse
import retrofit2.http.GET
import retrofit2.http.Query

import retrofit2.Response
import retrofit2.http.POST
import retrofit2.http.Path

interface AmrService {
    @GET("/amr/list")
    suspend fun getAmrList(): ApiResponse<List<AmrDto>>

    @GET("/amr/detail")
    suspend fun getAmrDetail(@Query("id") amrId: Long): ApiResponse<AmrDetailDto>

    @POST("amr/{id}/manual-start")
    suspend fun requestManualStart(@Path("id") id: Long): Response<Unit>

    @POST("amr/{id}/manual-return")
    suspend fun requestManualReturn(@Path("id") id: Long): Response<Unit>
}