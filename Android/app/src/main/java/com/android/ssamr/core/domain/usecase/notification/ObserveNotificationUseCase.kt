package com.android.ssamr.core.domain.usecase.notification

import com.android.ssamr.core.domain.repository.NotificationRepository
import javax.inject.Inject

class ObserveNotificationUseCase @Inject constructor(
    private val repository: NotificationRepository
){
    suspend operator fun invoke(id: Long) = repository.observeNotification(id)
}