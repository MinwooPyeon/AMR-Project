package com.android.ssamr.core.di

import com.android.ssamr.core.repository.AmrControlRepository
import com.android.ssamr.core.repository.AmrControlRepositoryImpl
import dagger.Binds
import dagger.Module
import dagger.hilt.InstallIn
import dagger.hilt.components.SingletonComponent
import javax.inject.Singleton

@Module
@InstallIn(SingletonComponent::class)
internal interface RepositoryModule {

    @Binds
    @Singleton
    fun bindAmrControlRepository(
        impl: AmrControlRepositoryImpl
    ): AmrControlRepository
}