package com.android.ssamr.core.data.repository

import com.android.ssamr.core.data.remote.datasource.AmrRemoteDataSource
import com.android.ssamr.core.domain.model.AmrAction
import com.android.ssamr.core.domain.model.AmrDetailAction
import com.android.ssamr.core.domain.model.AmrDetailStatus
import com.android.ssamr.core.domain.model.AmrStatus
import com.android.ssamr.core.domain.repository.AmrRepository
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
        return AmrDetailStatus("AMR-001", AmrDetailAction.RUNNING, "A-1구역", "1.4m/s", "자재이동", "RB-100", "RB100-2024-001", "v2.1.3", "111.111.1111")
//        return remoteDataSource.getAmrDetail(amrId)
    }

}