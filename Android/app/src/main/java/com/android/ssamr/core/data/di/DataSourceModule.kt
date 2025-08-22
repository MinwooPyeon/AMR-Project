package com.android.ssamr.core.data.di

import com.android.ssamr.core.data.local.dao.NotificationDao
import com.android.ssamr.core.data.local.datasource.NotificationLocalDataSource
import com.android.ssamr.core.data.remote.datasource.AmrRemoteDataSource
import com.android.ssamr.core.data.remote.service.AmrService
import com.android.ssamr.core.data.remote.datasource.DashboardRemoteDataSource
import com.android.ssamr.core.data.remote.datasource.FcmRemoteDataSource
import com.android.ssamr.core.data.remote.datasource.NotificationRemoteDataSource
import com.android.ssamr.core.data.remote.service.DashboardService
import com.android.ssamr.core.data.remote.service.FcmService
import com.android.ssamr.core.data.remote.service.NotificationService
import dagger.Module
import dagger.Provides
import dagger.hilt.InstallIn
import dagger.hilt.components.SingletonComponent
import javax.inject.Singleton

@Module
@InstallIn(SingletonComponent::class)
object DataSourceModule {

    @Provides
    @Singleton
    fun provideAmrRemoteDataSource(
        service: AmrService
    ): AmrRemoteDataSource = AmrRemoteDataSource(service)

    @Provides
    @Singleton
    fun provideDashboardRemoteDataSource(
        service: DashboardService
    ): DashboardRemoteDataSource = DashboardRemoteDataSource(service)

    @Provides
    @Singleton
    fun provideNotificationRemoteDataSource(
        service: NotificationService
    ): NotificationRemoteDataSource = NotificationRemoteDataSource(service)

    @Provides
    @Singleton
    fun provideNotificationLocalDataSource(
        dao: NotificationDao
    ): NotificationLocalDataSource = NotificationLocalDataSource(dao)

    @Provides
    @Singleton
    fun provideFcmRemoteDataSource(
        service: FcmService
    ): FcmRemoteDataSource = FcmRemoteDataSource(service)
}