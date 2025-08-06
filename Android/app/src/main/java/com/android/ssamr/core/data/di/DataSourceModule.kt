package com.android.ssamr.core.data.di

import com.android.ssamr.core.data.remote.datasource.AmrRemoteDataSource
import com.android.ssamr.core.data.remote.service.AmrService
<<<<<<< HEAD
import com.android.ssamr.core.data.remote.datasource.DashboardRemoteDataSource
import com.android.ssamr.core.data.remote.service.DashboardService
=======
>>>>>>> origin/develop
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
<<<<<<< HEAD

    @Provides
    @Singleton
    fun provideDashboardRemoteDataSource(
        service: DashboardService
    ): DashboardRemoteDataSource = DashboardRemoteDataSource(service)
=======
>>>>>>> origin/develop
}