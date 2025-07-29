package com.android.ssamr.core.domain.repository

import com.android.ssamr.feature.amr.AmrUiModel

interface AmrRepository {

    suspend fun getAmrList(): List<AmrUiModel>
}