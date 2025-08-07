package com.android.ssamr.core.data.di

import com.android.ssamr.core.data.repository.AmrRepositoryImpl
import com.android.ssamr.core.data.repository.DashboardRepositoryImpl
import com.android.ssamr.core.domain.repository.AmrRepository
import com.android.ssamr.core.domain.repository.DashboardRepository
import dagger.Binds
import dagger.Module
import dagger.hilt.InstallIn
import dagger.hilt.components.SingletonComponent
import javax.inject.Singleton

@Module
@InstallIn(SingletonComponent::class)
internal interface DataModule {

    @Binds
    @Singleton
    fun bindAmrRepository(
        amrRepository: AmrRepositoryImpl
    ): AmrRepository

    @Binds
    @Singleton
    fun bindDashboardRepository(
        dashboardRepository: DashboardRepositoryImpl
    ): DashboardRepository
}