package com.android.ssamr.core.domain.repository

import com.android.ssamr.core.domain.model.Notification
import kotlinx.coroutines.flow.Flow

interface NotificationRepository {
    /** Room: 로컬 알림 스트림 (UI는 이걸 observe) */
    fun observeNotifications(): Flow<List<Notification>>

    /** Server: 최신 알림 목록 가져오기 */
    suspend fun fetchNotificationsFromServer(): List<Notification>

    /** Room: 알림 목록 업서트(서버/FCM 동기화 결과 저장) */
    suspend fun upsertNotifications(list: List<Notification>)

    /** Room: 읽음 로컬 반영(즉시 UI 반영용) */
    suspend fun markReadLocal(id: Long)

    /** Server: 읽음 동기화(베스트에포트) */
    suspend fun markReadRemote(id: Long)
}
