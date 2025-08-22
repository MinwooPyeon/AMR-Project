package com.android.ssamr.core.domain.usecase.notification

import com.android.ssamr.core.domain.model.Notification
import com.android.ssamr.core.domain.repository.NotificationRepository
import javax.inject.Inject

class MarkNotificationReadLocalUseCase @Inject constructor(
    private val repository: NotificationRepository
){
    /** 클릭 즉시 로컬 읽음 처리 → UI 즉각 반영 */
    suspend operator fun invoke(id: Long) =
        repository.markReadLocal(id)
}