package com.android.ssamr.core.domain.repository

import com.android.ssamr.core.domain.model.AmrDetailStatus
import com.android.ssamr.core.domain.model.AmrStatus

interface AmrRepository {

    suspend fun getAmrList(): List<AmrStatus>

    suspend fun getAmrDetail(serial: String): AmrDetailStatus

    suspend fun manualControl(serial: String, area: String): Result<Unit>
}


