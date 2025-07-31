package com.android.ssamr.core.domain.repository

import com.android.ssamr.feature.amr.AmrUiModel
import com.android.ssamr.feature.amrDetail.AmrDetailUiModel

interface AmrRepository {

    suspend fun getAmrList(): List<AmrUiModel>

    suspend fun getAmrDetail(amrId: Long): AmrDetailUiModel

    suspend fun manualStart(id: Long): Result<Unit>
    suspend fun manualReturn(id: Long): Result<Unit>
}



