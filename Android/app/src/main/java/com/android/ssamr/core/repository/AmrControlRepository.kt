package com.android.ssamr.core.repository

import com.android.ssamr.core.network.api.AmrControlApi
import retrofit2.HttpException
import javax.inject.Inject

interface AmrControlRepository {
    suspend fun manualStart(id: Long): Result<Unit>
    suspend fun manualReturn(id: Long): Result<Unit>
}

class AmrControlRepositoryImpl @Inject constructor(
    private val api: AmrControlApi
) : AmrControlRepository {

    override suspend fun manualStart(id: Long): Result<Unit> = runCatching {
        val response = api.requestManualStart(id)
        if (response.isSuccessful) Unit else throw HttpException(response)
    }

    override suspend fun manualReturn(id: Long): Result<Unit> = runCatching {
        val response = api.requestManualReturn(id)
        if (response.isSuccessful) Unit else throw HttpException(response)
    }
}
