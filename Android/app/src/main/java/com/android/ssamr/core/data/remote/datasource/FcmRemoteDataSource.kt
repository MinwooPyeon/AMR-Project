package com.android.ssamr.core.data.remote.datasource

import com.android.ssamr.core.data.model.fcm.request.FcmTokenRequest
import com.android.ssamr.core.data.remote.service.FcmService
import retrofit2.Response
import javax.inject.Inject

class FcmRemoteDataSource @Inject constructor(
    private val fcmService: FcmService
) {
    suspend fun registerToken(token: String) : Response<Unit>
    = fcmService.registerFcmToken(FcmTokenRequest(token))
}