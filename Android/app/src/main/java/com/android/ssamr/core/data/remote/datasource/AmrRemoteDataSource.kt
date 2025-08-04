package com.android.ssamr.core.data.remote.datasource

import com.android.ssamr.core.data.model.amr.response.toDetailModel
import com.android.ssamr.core.data.model.amr.response.toUiModel
import com.android.ssamr.core.data.remote.service.AmrService
import com.android.ssamr.core.domain.model.AmrDetailAction
import com.android.ssamr.core.domain.model.AmrDetailStatus
import com.android.ssamr.core.domain.model.AmrStatus
import retrofit2.Response
import javax.inject.Inject


class AmrRemoteDataSource @Inject constructor(
    private val service: AmrService
) {
    suspend fun getAmrList(): List<AmrStatus> {
        val response = service.getAmrList()

        return response.data?.map { it.toUiModel() } ?: emptyList()
    }

    suspend fun getAmrDetail(amrId: Long): AmrDetailStatus  {
        val response = service.getAmrDetail(amrId)

        return response.data?.toDetailModel() ?: throw IllegalStateException("상세정보 없음")
    }

    suspend fun manualStart(amrId: Long): Response<Unit> {
        return service.requestManualStart(amrId)
    }

    suspend fun manualReturn(amrId: Long): Response<Unit> {
        return service.requestManualReturn(amrId)
    }
}