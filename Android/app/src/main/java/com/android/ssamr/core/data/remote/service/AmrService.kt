package com.android.ssamr.core.data.remote.service

import com.android.ssamr.core.data.model.amr.request.ManualControlRequest
import com.android.ssamr.core.data.model.amr.response.AmrDetailDto
import com.android.ssamr.core.data.model.amr.response.AmrDto
import retrofit2.http.GET
import retrofit2.http.POST
import retrofit2.http.Path
import retrofit2.Response
import retrofit2.http.Body

interface AmrService {
    @GET("api/v1/amrs/latest-statuses")
    suspend fun getAmrList(): List<AmrDto>

    @GET("api/v1/amrs/{id}/detail")
    suspend fun getAmrDetail(@Path("id") amrId: Long): AmrDetailDto

    @POST("api/v1/amrs/{id}/control")
    suspend fun requestControl( @Path("id") id: Long,
        @Body request: ManualControlRequest): Response<Unit>
}