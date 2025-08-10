package com.android.ssamr.core.domain.usecase.notification

import com.android.ssamr.core.domain.model.Notification
import com.android.ssamr.core.domain.repository.NotificationRepository
import kotlinx.coroutines.flow.Flow
import javax.inject.Inject

class GetLocalNotificationFlowUseCase @Inject constructor(
    private val repository: NotificationRepository
){
    operator fun invoke(): Flow<List<Notification>> = repository.observeNotifications()

}