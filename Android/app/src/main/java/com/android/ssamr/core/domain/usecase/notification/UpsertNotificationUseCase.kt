package com.android.ssamr.core.domain.usecase.notification

import com.android.ssamr.core.domain.model.Notification
import com.android.ssamr.core.domain.repository.NotificationRepository
import javax.inject.Inject

class UpsertNotificationUseCase @Inject constructor(
    private val repository: NotificationRepository
) {
    suspend operator fun invoke(item: Notification) = repository.upsertNotification(item)
}