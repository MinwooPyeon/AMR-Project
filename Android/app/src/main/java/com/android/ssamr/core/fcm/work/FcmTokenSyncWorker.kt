// com.android.ssamr.core.fcm.work.FcmTokenSyncWorker
package com.android.ssamr.core.fcm.work

import android.content.Context
import androidx.work.CoroutineWorker
import androidx.work.WorkerParameters
import com.android.ssamr.core.domain.usecase.push.RegisterFcmTokenUseCase
import dagger.hilt.EntryPoint
import dagger.hilt.InstallIn
import dagger.hilt.android.EntryPointAccessors
import dagger.hilt.components.SingletonComponent

class FcmTokenSyncWorker(
    appContext: Context,
    params: WorkerParameters
) : CoroutineWorker(appContext, params) {

    override suspend fun doWork(): Result {
        val token = inputData.getString(KEY_TOKEN) ?: return Result.failure()

        // Hilt EntryPoint로 필요한 의존성 꺼내 쓰기 (Worker는 Hilt가 생성하지 않아도 됨)
        val entry = EntryPointAccessors.fromApplication(
            applicationContext,
            FcmWorkerEntryPoint::class.java
        )
        val register: RegisterFcmTokenUseCase = entry.registerFcmTokenUseCase()

        return runCatching {
            register(token) // 서버로 등록 (suspend)
            Result.success()
        }.getOrElse {
            if (runAttemptCount < 5) Result.retry() else Result.failure()
        }
    }

    companion object { const val KEY_TOKEN = "token" }
}

/** Worker에서 Hilt 의존성에 접근하기 위한 EntryPoint */
@EntryPoint
@InstallIn(SingletonComponent::class)
interface FcmWorkerEntryPoint {
    fun registerFcmTokenUseCase(): RegisterFcmTokenUseCase
}
