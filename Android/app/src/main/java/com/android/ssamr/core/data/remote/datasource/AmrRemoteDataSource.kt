package com.android.ssamr.core.data.remote.datasource

import com.android.ssamr.core.data.model.amr.response.toDetailModel
import com.android.ssamr.core.data.model.amr.response.toUiModel
import com.android.ssamr.core.data.remote.service.AmrService
import com.android.ssamr.feature.amr.AmrUiModel
import com.android.ssamr.feature.amrDetail.AmrDetailUiModel
import javax.inject.Inject

class AmrRemoteDataSource @Inject constructor(
    private val service: AmrService
) {
    suspend fun getAmrList(): List<AmrUiModel> {
        val response = service.getAmrList()

        return response.map { it.toUiModel() }
    }

    suspend fun getAmrDetail(amrId: Long): AmrDetailUiModel {
        val response = service.getAmrDetail(amrId)

        return response.toDetailModel()
    }
}