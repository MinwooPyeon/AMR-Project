package com.android.ssamr.core.data.repository

import com.android.ssamr.core.data.remote.datasource.AmrRemoteDataSource
import com.android.ssamr.core.domain.model.AmrAction
import com.android.ssamr.core.domain.model.AmrDetailAction
import com.android.ssamr.core.domain.model.AmrDetailStatus
import com.android.ssamr.core.domain.model.AmrStatus
import com.android.ssamr.core.domain.repository.AmrRepository
import retrofit2.HttpException
import javax.inject.Inject

class AmrRepositoryImpl @Inject constructor(
    private val remoteDataSource: AmrRemoteDataSource
) : AmrRepository {

    override suspend fun getAmrList(): List<AmrStatus> {
        return listOf(
            AmrStatus(1L, "1번로봇", AmrAction.RUNNING, "A-1구역", "1.4m/s", "자재이동"),
            AmrStatus(2L, "2번로봇", AmrAction.CHARGING, "충전소", "0.0m/s", "대기"),
            AmrStatus(3L, "3번로봇", AmrAction.CHECKING, "B-3구역", "0.0m/s", "점검")
        )
//        return remoteDataSource.getAmrList()
    }

    override suspend fun getAmrDetail(amrId: Long): AmrDetailStatus {
//        return remoteDataSource.getAmrDetail(amrId)
        return AmrDetailStatus (
            name = "AMR-${String.format("%03d", amrId)}",
            status = AmrDetailAction.RUNNING,
            location = "A구역-라인${amrId}",
            speed = "1.2m/s",
            job = "화물 운반 중",
            model = "RB-100",
            serial = "RB100-2024-${String.format("%03d", amrId)}",
            firmware = "v2.1.3",
            ipAddress = "192.168.x.x"
        )
    }

    override suspend fun manualStart(id: Long): Result<Unit> = runCatching {
        val response = remoteDataSource.manualStart(id)
        if (response.isSuccessful) Unit else throw HttpException(response)
    }

    override suspend fun manualReturn(id: Long): Result<Unit> = runCatching {
        val response = remoteDataSource.manualReturn(id)
        if (response.isSuccessful) Unit else throw HttpException(response)
    }
}

