package com.android.ssamr.core.data.local

import androidx.room.Database
import androidx.room.RoomDatabase
import com.android.ssamr.core.data.local.dao.NotificationDao
import com.android.ssamr.core.data.local.entity.NotificationEntity

@Database(
    entities = [NotificationEntity::class],
    version = 1,
    exportSchema = false
)
abstract class AppDatabase : RoomDatabase() {
    abstract fun notificationDao(): NotificationDao
}