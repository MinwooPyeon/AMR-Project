package com.android.ssamr.core.domain.usecase.amr

import com.android.ssamr.core.domain.repository.AmrRepository
import com.android.ssamr.feature.amrDetail.AmrDetailUiModel
import javax.inject.Inject

class GetAmrDetailUseCase @Inject constructor(
    private val amrRepository: AmrRepository
) {
    suspend operator fun invoke(amrId: Long): AmrDetailUiModel {
        return amrRepository.getAmrDetail(amrId)
    }
}