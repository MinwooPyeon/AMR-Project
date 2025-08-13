package com.android.ssamr.core.data.local.datasource

import com.android.ssamr.core.data.local.dao.NotificationDao
import com.android.ssamr.core.data.local.entity.NotificationEntity
import javax.inject.Inject

class NotificationLocalDataSource @Inject constructor(
    private val dao: NotificationDao
) {
    fun observeNotifications() = dao.observeAll()

    suspend fun upsertNotifications(list: List<NotificationEntity>) = dao.upsertAll(list)

    suspend fun markRead(id: Long) = dao.markRead(id)

    suspend fun clear() = dao.clear()

    fun observeById(id: Long) = dao.observeById(id)
}