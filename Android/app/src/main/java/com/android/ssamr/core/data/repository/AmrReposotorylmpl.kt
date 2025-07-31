package com.android.ssamr.core.data.repository

import com.android.ssamr.core.data.remote.datasource.AmrRemoteDataSource
import com.android.ssamr.core.domain.repository.AmrRepository
import com.android.ssamr.feature.amr.AmrStatus
import com.android.ssamr.feature.amr.AmrUiModel
import com.android.ssamr.feature.amrDetail.AmrDetailUiModel
import retrofit2.HttpException
import javax.inject.Inject

class AmrRepositoryImpl @Inject constructor(
    private val remoteDataSource: AmrRemoteDataSource
) : AmrRepository {

    override suspend fun getAmrList(): List<AmrUiModel> {
        return listOf(
            AmrUiModel(1L, "1번로봇", AmrStatus.RUNNING, "A-1구역", "1.4m/s", "자재이동", 92),
            AmrUiModel(2L, "2번로봇", AmrStatus.CHARGING, "충전소", "0.0m/s", "대기", 54),
            AmrUiModel(3L, "3번로봇", AmrStatus.CHECK, "B-3구역", "0.0m/s", "점검", 80)
        )
//        return remoteDataSource.getAmrList()
    }

    override suspend fun getAmrDetail(amrId: Long): AmrDetailUiModel {
        // return remoteDataSource.getAmrDetail(amrId)
        return AmrDetailUiModel(
            name = "AMR-00$amrId",
            status = "대기중",
            battery = 78,
            location = "B-1 구역",
            speed = "1.0m/s",
            job = "화물 대기",
            model = "RB-100",
            serial = "RB100-TEST-$amrId",
            firmware = "v1.0.0"
        )
    }


    override suspend fun manualStart(amrId: Long): Result<Unit> = runCatching {
        val response = remoteDataSource.manualStart(amrId)
        if (response.isSuccessful) Unit else throw HttpException(response)
    }

    override suspend fun manualReturn(amrId: Long): Result<Unit> = runCatching {
        val response = remoteDataSource.manualReturn(amrId)
        if (response.isSuccessful) Unit else throw HttpException(response)
    }
}
