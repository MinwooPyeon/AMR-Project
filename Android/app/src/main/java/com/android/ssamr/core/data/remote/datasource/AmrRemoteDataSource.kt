package com.android.ssamr.core.data.remote.datasource

import com.android.ssamr.core.data.model.amr.response.toDetailModel
import com.android.ssamr.core.data.model.amr.response.toUiModel
import com.android.ssamr.core.data.remote.service.AmrService
import com.android.ssamr.feature.amr.AmrUiModel
import com.android.ssamr.feature.amrDetail.AmrDetailUiModel
import retrofit2.Response
import javax.inject.Inject

class AmrRemoteDataSource @Inject constructor(
    private val service: AmrService
) {
    suspend fun getAmrList(): List<AmrUiModel> {
        val response = service.getAmrList()

        return response.data?.map { it.toUiModel() } ?: emptyList()
    }

    suspend fun getAmrDetail(amrId: Long): AmrDetailUiModel {
        val response = service.getAmrDetail(amrId)

        return response.data?.toDetailModel() ?: throw IllegalStateException("상세정보 없음")
    }

    suspend fun manualStart(id: Long): Response<Unit> {
        return service.requestManualStart(id)
    }

    suspend fun manualReturn(id: Long): Response<Unit> {
        return service.requestManualReturn(id)
    }
}