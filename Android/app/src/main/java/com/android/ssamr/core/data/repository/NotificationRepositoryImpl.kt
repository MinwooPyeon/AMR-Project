package com.android.ssamr.core.data.repository

import com.android.ssamr.core.data.local.datasource.NotificationLocalDataSource
import com.android.ssamr.core.data.mapper.NotificationMapper
import com.android.ssamr.core.data.remote.datasource.NotificationRemoteDataSource
import com.android.ssamr.core.domain.model.Notification
import com.android.ssamr.core.domain.repository.NotificationRepository
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.first
import kotlinx.coroutines.flow.map
import javax.inject.Inject

class NotificationRepositoryImpl @Inject constructor(
    private val remote: NotificationRemoteDataSource,
    private val local: NotificationLocalDataSource
) : NotificationRepository {
    override fun observeNotifications(): Flow<List<Notification>> =
        local.observeNotifications().map { entities ->
            entities.map { NotificationMapper.fromEntity(it) }
        }


    override suspend fun fetchNotificationsFromServer(): List<Notification> =
        remote.fetchNotifications().map { NotificationMapper.dtoToDomain(it) }


    override suspend fun upsertNotifications(list: List<Notification>) {
        val entities = NotificationMapper.domainListToEntities(list)
        local.upsertNotifications(entities)
    }

    override suspend fun markReadLocal(id: Long) {
        local.markRead(id)
    }

    override suspend fun markReadRemote(id: Long) {
        remote.markRead(id)
    }

    suspend fun syncWithMerge() {
        val server = fetchNotificationsFromServer()
        val localSnapshot = observeNotifications().first()
        val localReadMap = localSnapshot.associate { it.id to it.isRead }

        val merged = server.map { s ->
            if (localReadMap[s.id] == true) s.copy(isRead = true) else s
        }
        upsertNotifications(merged)
    }
}
