package com.android.ssamr.core.data.local.dao

import androidx.room.Dao
import androidx.room.Insert
import androidx.room.OnConflictStrategy
import androidx.room.Query
import com.android.ssamr.core.data.local.entity.NotificationEntity
import kotlinx.coroutines.flow.Flow

@Dao
interface NotificationDao {
    @Query("SELECT * FROM notifications ORDER BY date DESC")
    fun observeAll(): Flow<List<NotificationEntity>>

    @Insert(onConflict = OnConflictStrategy.REPLACE)
    suspend fun upsertAll(list: List<NotificationEntity>)

    @Query("UPDATE notifications SET isRead = 1, readAt = :readAt WHERE id = :id")
    suspend fun markRead(id: Long, readAt: Long? = System.currentTimeMillis())

    @Query("DELETE FROM notifications")
    suspend fun clear()
}
