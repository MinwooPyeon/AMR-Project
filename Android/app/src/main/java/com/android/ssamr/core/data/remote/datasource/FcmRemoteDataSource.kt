package com.android.ssamr.core.data.remote.datasource

import com.android.ssamr.core.data.remote.service.FcmService
import javax.inject.Inject

class FcmRemoteDataSource @Inject constructor(
    private val fcmService: FcmService
) {
    suspend fun registerToken(token: String) = fcmService.registerFcmToken(token)
}