package com.android.ssamr.core.data.remote.datasource

import com.android.ssamr.core.data.model.amr.response.toUiModel
import com.android.ssamr.core.data.remote.service.AmrService
import com.android.ssamr.feature.amr.AmrUiModel
import javax.inject.Inject

class AmrRemoteDataSource @Inject constructor(
    private val service: AmrService
) {
    suspend fun getAmrList(): List<AmrUiModel> {
        val response = service.getAmrList()

        return response.data?.map { it.toUiModel() } ?: emptyList()
    }
}