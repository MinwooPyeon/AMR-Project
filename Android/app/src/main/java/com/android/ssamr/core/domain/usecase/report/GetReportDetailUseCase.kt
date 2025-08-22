package com.android.ssamr.core.domain.usecase.report

import com.android.ssamr.core.data.mapper.ReportMapper
import com.android.ssamr.core.domain.model.Report
import com.android.ssamr.core.domain.repository.NotificationRepository
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.map
import javax.inject.Inject

class GetReportDetailUseCase @Inject constructor(
    private val repository: NotificationRepository
) {
    suspend operator fun invoke(id: Long): Flow<Report?> =
        repository.observeNotification(id).map { n -> n?.let(ReportMapper::notificationToReport) }
}