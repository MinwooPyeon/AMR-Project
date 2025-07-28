package com.android.ssamr.core.data.repository

import com.android.ssamr.core.data.remote.AmrApi
import com.android.ssamr.core.data.remote.ManualCommandRequest
import com.android.ssamr.core.domain.AmrCommandRepository
import javax.inject.Inject


class AmrCommandRepositoryImpl @Inject constructor(
    private val amrApi: AmrApi
) : AmrCommandRepository {
    override suspend fun sendManualStartCommand(amrId: Long): Result<Unit> {
        return try {
            val response = amrApi.manualStart(ManualCommandRequest(amrId))
            if (response.isSuccessful) Result.success(Unit)
            else Result.failure(Exception("출발 실패: ${response.code()}"))
        } catch (e: Exception) {
            Result.failure(e)
        }
    }

    override suspend fun sendManualReturnCommand(amrId: Long): Result<Unit> {
        return try {
            val response = amrApi.manualReturn(ManualCommandRequest(amrId))
            if (response.isSuccessful) Result.success(Unit)
            else Result.failure(Exception("복귀 실패: ${response.code()}"))
        } catch (e: Exception) {
            Result.failure(e)
        }
    }
}
