package com.android.ssamr.core.domain.usecase.amr

import com.android.ssamr.core.domain.repository.AmrRepository
import javax.inject.Inject

class ManualControlUseCase @Inject constructor(
    private val repository: AmrRepository
) {
    suspend operator fun invoke(amrId: Long, destination: String): Result<Unit> {
        return repository.manualControl(amrId, destination)
    }
}