package com.android.ssamr.core.domain.usecase.amr

import com.android.ssamr.core.domain.repository.AmrRepository
import com.android.ssamr.feature.amr.AmrUiModel
import javax.inject.Inject

class GetAmrListUseCase @Inject constructor(
    private val amrRepository: AmrRepository
) {
    suspend operator fun invoke(): List<AmrUiModel> {
        return amrRepository.getAmrList()
    }
}