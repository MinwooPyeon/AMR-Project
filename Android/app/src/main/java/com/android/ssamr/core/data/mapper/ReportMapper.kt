package com.android.ssamr.core.data.mapper

import com.android.ssamr.core.domain.model.Notification
import com.android.ssamr.core.domain.model.Report

object ReportMapper {

    fun notificationToReport(n: Notification): Report {
        val safeCase = n.case.uppercase()
        return Report(
            id = n.id,
            title = n.title,
            content = n.content,
            riskLevel = n.riskLevel,
            area = n.area,
            case = safeCase,
            createAt = n.createAt,
            image = n.image,
            serial = n.serial
        )

    }
}