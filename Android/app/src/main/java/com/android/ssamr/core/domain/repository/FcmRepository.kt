package com.android.ssamr.core.domain.repository

import retrofit2.Response

interface FcmRepository {
    suspend fun registerToken(token: String): Response<Unit>
}