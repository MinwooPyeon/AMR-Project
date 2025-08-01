package com.android.ssamr.core.domain.usecase.amr

import com.android.ssamr.core.domain.model.AmrDetailStatus
import com.android.ssamr.core.domain.repository.AmrRepository
import javax.inject.Inject

class GetAmrDetailUseCase @Inject constructor(
    private val amrRepository: AmrRepository
) {
    suspend operator fun invoke(amrId: Long): AmrDetailStatus {
        return amrRepository.getAmrDetail(amrId)
    }
}