package com.android.ssamr.core.domain.usecase.notification

import com.android.ssamr.core.domain.model.Notification
import com.android.ssamr.core.domain.repository.NotificationRepository
import javax.inject.Inject

class GetNotificationListUseCase @Inject constructor(
    private val repository: NotificationRepository
) {
    /** 서버 최신 목록 가져오기 (저장은 별도 UseCase에서) */
    suspend operator fun invoke(): List<Notification> =
        repository.fetchNotificationsFromServer()
}