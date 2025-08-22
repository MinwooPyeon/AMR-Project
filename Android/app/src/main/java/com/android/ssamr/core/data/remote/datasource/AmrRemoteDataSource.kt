package com.android.ssamr.core.data.remote.datasource

import com.android.ssamr.core.data.model.amr.request.ManualControlRequest
import com.android.ssamr.core.data.model.amr.response.toDetailModel
import com.android.ssamr.core.data.model.amr.response.toUiModel
import com.android.ssamr.core.data.remote.service.AmrService
import com.android.ssamr.core.domain.model.AmrDetailStatus
import com.android.ssamr.core.domain.model.AmrStatus
import retrofit2.Response
import javax.inject.Inject

class AmrRemoteDataSource @Inject constructor(
    private val service: AmrService
) {
    suspend fun getAmrList(): List<AmrStatus> {
        val response = service.getAmrList()

        return response.map { it.toUiModel() }
    }

    suspend fun getAmrDetail(serial: String): AmrDetailStatus {

        val response = service.getAmrDetail(serial)

        return response.toDetailModel()
    }


    suspend fun manualControl(serial: String, area: String): Response<Unit> {
        return service.requestControl(ManualControlRequest(serial, area))
    }
}