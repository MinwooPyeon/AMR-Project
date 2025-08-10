package com.android.ssamr.core.data.di

import android.content.Context
import androidx.room.Room
import com.android.ssamr.core.data.local.AppDatabase
import com.android.ssamr.core.data.local.dao.NotificationDao
import dagger.Module
import dagger.Provides
import dagger.hilt.InstallIn
import dagger.hilt.android.qualifiers.ApplicationContext
import dagger.hilt.components.SingletonComponent
import javax.inject.Singleton

@Module
@InstallIn(SingletonComponent::class)
object DatabaseModule {

    @Provides
    @Singleton
    fun provideAppDatabase(
        @ApplicationContext context: Context
    ): AppDatabase = Room.databaseBuilder(
        context,
        AppDatabase::class.java,
        "ssamr.db"
    ).fallbackToDestructiveMigration().build()

    @Provides
    @Singleton
    fun provideNotificationDao(db: AppDatabase): NotificationDao = db.notificationDao()

}