package com.android.ssamr.core.domain.usecase.amr

import com.android.ssamr.core.domain.model.DashboardAmr
import com.android.ssamr.core.domain.repository.DashboardRepository
import com.android.ssamr.core.data.model.amr.response.toDashboardModel
import javax.inject.Inject

class GetDashboardAmrsUseCase @Inject constructor(
    private val repository: DashboardRepository
) {
    suspend operator fun invoke(): Result<List<DashboardAmr>> = runCatching {
        val dtos = repository.getDashboardAmrs()
        dtos.map { it.toDashboardModel() }
    }
}