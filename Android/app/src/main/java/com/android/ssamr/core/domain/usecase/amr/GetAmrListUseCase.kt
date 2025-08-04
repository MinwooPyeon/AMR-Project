package com.android.ssamr.core.domain.usecase.amr

import com.android.ssamr.core.domain.model.AmrDetailAction
import com.android.ssamr.core.domain.model.AmrStatus
import com.android.ssamr.core.domain.repository.AmrRepository
import javax.inject.Inject

class GetAmrListUseCase @Inject constructor(
    private val amrRepository: AmrRepository
) {
    suspend operator fun invoke(): List<AmrStatus> {
        return amrRepository.getAmrList()
    }
}