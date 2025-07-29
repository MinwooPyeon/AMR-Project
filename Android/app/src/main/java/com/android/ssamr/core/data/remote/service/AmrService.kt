package com.android.ssamr.core.data.remote.service

import com.android.ssamr.core.data.model.amr.response.AmrDto
import com.android.ssamr.core.network.model.ApiResponse
import retrofit2.http.GET

interface AmrService {
    @GET("/amr/list")
    suspend fun getAmrList(): ApiResponse<List<AmrDto>>
}