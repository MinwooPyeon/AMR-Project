package com.android.ssamr.core.domain.usecase.push

import androidx.work.BackoffPolicy
import androidx.work.Constraints
import androidx.work.ExistingWorkPolicy
import androidx.work.NetworkType
import androidx.work.OneTimeWorkRequestBuilder
import androidx.work.WorkManager
import androidx.work.workDataOf
import com.android.ssamr.core.fcm.work.FcmTokenSyncWorker
import java.util.concurrent.TimeUnit
import javax.inject.Inject

class EnqueueRegisterFcmTokenUseCase @Inject constructor(
    private val workManager: WorkManager
) {
    operator fun invoke(token: String) {
        val request = OneTimeWorkRequestBuilder<FcmTokenSyncWorker>()
            .setInputData(workDataOf(FcmTokenSyncWorker.KEY_TOKEN to token))
            .setConstraints(
                Constraints.Builder().setRequiredNetworkType(NetworkType.CONNECTED).build()
            )
            .setBackoffCriteria(BackoffPolicy.EXPONENTIAL, 30, TimeUnit.SECONDS)
            .build()

        workManager.enqueueUniqueWork(
            "sync_fcm_token", ExistingWorkPolicy.REPLACE, request
        )
    }
}