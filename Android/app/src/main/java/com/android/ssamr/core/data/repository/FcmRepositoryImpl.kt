package com.android.ssamr.core.data.repository

import com.android.ssamr.core.data.remote.datasource.FcmRemoteDataSource
import com.android.ssamr.core.domain.repository.FcmRepository
import javax.inject.Inject

class FcmRepositoryImpl @Inject constructor(
    private val fcmRemoteDataSource: FcmRemoteDataSource
) : FcmRepository {
    override suspend fun registerToken(token: String) {
        fcmRemoteDataSource.registerToken(token)
    }
}