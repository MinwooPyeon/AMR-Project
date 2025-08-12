package com.android.ssamr.core.domain.usecase.notification

import com.android.ssamr.core.domain.repository.NotificationRepository
import javax.inject.Inject

class MarkNotificationReadRemoteUseCase @Inject constructor(
    private val repository: NotificationRepository
) {
    /** 서버에 읽음 동기화 (실패는 베스트에포트 처리 가능) */
    suspend operator fun invoke(id: Long) = repository.markReadRemote(id)
}