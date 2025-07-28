package com.android.ssamr.core.usecase.amr

import kotlinx.coroutines.delay
import javax.inject.Inject

class LoginUseCase @Inject constructor() {

    suspend operator fun invoke(email: String, password: String): Result<Unit> {
        delay(1000) // 예시: 서버 대기 시뮬레이션

        return if (email == "admin" && password == "1234") {
            Result.success(Unit)
        } else {
            Result.failure(Exception("이메일 또는 비밀번호가 틀렸습니다."))
        }
    }
}