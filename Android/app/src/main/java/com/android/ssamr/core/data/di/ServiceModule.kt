package com.android.ssamr.core.data.di

import com.android.ssamr.core.data.remote.service.AmrService
import com.android.ssamr.core.data.remote.service.DashboardService
import com.android.ssamr.core.data.remote.service.NotificationService
import dagger.Module
import dagger.Provides
import dagger.hilt.InstallIn
import dagger.hilt.components.SingletonComponent
import retrofit2.Retrofit
import javax.inject.Singleton

@Module
@InstallIn(SingletonComponent::class)
object ServiceModule {

    @Provides
    @Singleton
    fun provideAmrService(retrofit: Retrofit): AmrService {
        return retrofit.create(AmrService::class.java)
    }

    @Provides
    @Singleton
    fun provideDashboardService(retrofit: Retrofit): DashboardService {
        return retrofit.create(DashboardService::class.java)
    }

    @Provides
    @Singleton
    fun provideNotificationService(retrofit: Retrofit): NotificationService =
        retrofit.create(NotificationService::class.java)


}