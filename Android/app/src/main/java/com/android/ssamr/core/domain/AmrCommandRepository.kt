package com.android.ssamr.core.domain

interface AmrCommandRepository {
    suspend fun sendManualStartCommand(amrId: Long): Result<Unit>
    suspend fun sendManualReturnCommand(amrId: Long): Result<Unit>
}