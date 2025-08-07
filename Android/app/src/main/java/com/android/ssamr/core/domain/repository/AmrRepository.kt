package com.android.ssamr.core.domain.repository

import com.android.ssamr.core.domain.model.AmrDetailAction
import com.android.ssamr.core.domain.model.AmrDetailStatus
import com.android.ssamr.core.domain.model.AmrStatus

interface AmrRepository {

    suspend fun getAmrList(): List<AmrStatus>

    suspend fun getAmrDetail(amrId: Long): AmrDetailStatus

    suspend fun manualControl(amrId: Long, destination: String): Result<Unit>
}
