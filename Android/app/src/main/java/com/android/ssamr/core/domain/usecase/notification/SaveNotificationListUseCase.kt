package com.android.ssamr.core.domain.usecase.notification

import com.android.ssamr.core.domain.model.Notification
import com.android.ssamr.core.domain.repository.NotificationRepository
import javax.inject.Inject

class SaveNotificationListUseCase @Inject constructor(
    private val repository: NotificationRepository
){
    /** Room에 업서트 */
    suspend operator fun invoke(list: List<Notification>) =
        repository.upsertNotifications(list)
}