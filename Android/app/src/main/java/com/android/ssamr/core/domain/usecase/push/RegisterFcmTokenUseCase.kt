package com.android.ssamr.core.domain.usecase.push

import com.android.ssamr.core.domain.repository.FcmRepository
import retrofit2.Response
import javax.inject.Inject

class RegisterFcmTokenUseCase @Inject constructor(
    private val repo: FcmRepository
) {
    suspend operator fun invoke(token: String) : Response<Unit> =
        repo.registerToken(token)
}