package com.android.ssamr.core.domain.usecase.report

import com.android.ssamr.core.data.mapper.ReportMapper
import com.android.ssamr.core.domain.model.Report
import com.android.ssamr.core.domain.repository.NotificationRepository
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.map
import javax.inject.Inject

class GetReportListUseCase @Inject constructor(
    private val notificationRepository: NotificationRepository
) {
    operator fun invoke(): Flow<List<Report>> =
        notificationRepository.observeNotifications()
            .map { notis -> notis.map(ReportMapper::notificationToReport) }
}