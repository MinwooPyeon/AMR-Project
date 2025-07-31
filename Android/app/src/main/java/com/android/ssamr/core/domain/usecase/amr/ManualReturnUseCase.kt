package com.android.ssamr.core.domain.usecase.amr

import com.android.ssamr.core.domain.repository.AmrRepository
import javax.inject.Inject

class ManualReturnUseCase @Inject constructor(
    private val repository: AmrRepository
) {
    suspend operator fun invoke(id: Long): Result<Unit> {
        return repository.manualReturn(id)
    }
}

