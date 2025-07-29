package com.android.ssamr.core.data.di

import com.android.ssamr.core.data.remote.datasource.AmrRemoteDataSource
import com.android.ssamr.core.data.remote.service.AmrService
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
}