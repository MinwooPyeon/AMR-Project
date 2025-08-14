package com.android.ssamr.core.domain.repository

interface FcmRepository {
    suspend fun registerToken(token: String)
}