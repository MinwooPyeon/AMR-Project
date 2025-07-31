package com.android.ssamr.core.data.remote.service

import com.android.ssamr.core.data.model.amr.response.AmrDetailDto
import com.android.ssamr.core.data.model.amr.response.AmrDto
import retrofit2.http.GET
import retrofit2.http.Query

interface AmrService {
    @GET("api/v1/amrs/latest-statuses")
    suspend fun getAmrList(): List<AmrDto>

    @GET("/amr/detail")
    suspend fun getAmrDetail(@Query("id") amrId: Long): AmrDetailDto
}