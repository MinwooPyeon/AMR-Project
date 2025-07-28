package com.android.ssamr.core.usecase.amr

import com.android.ssamr.core.repository.AmrControlRepository
import javax.inject.Inject

class ManualStartUseCase @Inject constructor(
    private val repository: AmrControlRepository
) {
    suspend operator fun invoke(id: Long): Result<Unit> {
        return repository.manualStart(id)
    }
}