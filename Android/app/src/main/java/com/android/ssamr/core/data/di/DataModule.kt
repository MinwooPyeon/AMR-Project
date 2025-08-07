package com.android.ssamr.core.data.di

import com.android.ssamr.core.data.repository.AmrRepositoryImpl
import com.android.ssamr.core.domain.repository.AmrRepository
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
}