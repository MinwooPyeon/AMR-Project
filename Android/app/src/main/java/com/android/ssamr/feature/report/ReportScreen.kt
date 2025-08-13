package com.android.ssamr.feature.report

import android.os.Build
import androidx.annotation.RequiresApi
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.runtime.Composable
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.tooling.preview.Preview
import androidx.compose.ui.unit.dp
import com.android.ssamr.core.domain.model.NotificationAction
import com.android.ssamr.core.domain.model.Report
import com.android.ssamr.core.domain.model.ReportAction
import com.android.ssamr.core.domain.model.ReportCategory
import com.android.ssamr.ui.theme.SSAMRTheme

@RequiresApi(Build.VERSION_CODES.O)
@Composable
fun ReportScreen(
    state: ReportState,
    sendIntent: (ReportIntent) -> Unit = {}
) {
    Box(
        modifier = Modifier.fillMaxSize()
    ) {
        Column(modifier = Modifier.fillMaxSize()) {
            Column(verticalArrangement = Arrangement.spacedBy(12.dp), modifier = Modifier.padding(12.dp)) {
                Row(horizontalArrangement = Arrangement.spacedBy(12.dp), modifier = Modifier.fillMaxWidth()) {
                    ReportStatCard(count = state.startCardCounts[ReportCategory.ALL], label = "총 알림", countColor = Color.Black, modifier = Modifier.weight(1f))
                }
                Row(horizontalArrangement = Arrangement.spacedBy(12.dp), modifier = Modifier.fillMaxWidth()) {
                    ReportStatCard(count = state.startCardCounts[ReportCategory.COLLAPSE], label = "적재 사고", countColor = Color.Blue, modifier = Modifier.weight(1f))
                    ReportStatCard(count = state.startCardCounts[ReportCategory.SMOKE], label = "공장 내 흡연", countColor = Color(0xFF388E3C), modifier = Modifier.weight(1f))
                }
                Row(horizontalArrangement = Arrangement.spacedBy(12.dp), modifier = Modifier.fillMaxWidth()) {
                    ReportStatCard(count = state.startCardCounts[ReportCategory.EQUIPMENT], label = "안전 장비 미착용", countColor = Color(0xFFD32F2F), modifier = Modifier.weight(1f))
                    ReportStatCard(count = state.startCardCounts[ReportCategory.DANGER], label = "인명사고", countColor = Color(0xFFF57C00), modifier = Modifier.weight(1f))
                }
            }
            ReportCategoryTabRow(state = state, sendIntent = sendIntent)

            Spacer(Modifier.height(8.dp))

            ReportCardList(
                reports = state.reportList,
                onCardClick = { reportId ->
                    sendIntent(ReportIntent.ClickReportCard(reportId))
                }
            )
        }

    }
}

@Preview(showBackground = true)
@Composable
fun ReportScreenPreview() {
    SSAMRTheme {
        ReportScreen(
            state = ReportState(
                startCardCounts = mapOf(
                    ReportCategory.ALL to 10,
                    ReportCategory.COLLAPSE to 5,
                    ReportCategory.SMOKE to 3,
                    ReportCategory.EQUIPMENT to 2,
                    ReportCategory.DANGER to 1
                ),
                reportList = sampleReports,
                fullReportList = sampleReports,
                selectedCategory = ReportCategory.ALL,
            )
        ) {}
    }
}

val sampleReports = listOf(
    Report(
        id = 1L,
        title = "창고 A구역 화재 위험 감지",
        content = "…",
        riskLevel = NotificationAction.DANGER,
        area = "A-12",
        case = "화재",
        image = null,
        serial = "AMR-001",
        createAt = "2024-01-15 14:30",
    ),
    Report(
        id = 1L,
        title = "창고 A구역 화재 위험 감지",
        content = "…",
        riskLevel = NotificationAction.DANGER,
        area = "A-12",
        case = "화재",
        image = null,
        serial = "AMR-001",
        createAt = "2024-01-15 14:30",
    ),
    Report(
        id = 1L,
        title = "창고 A구역 화재 위험 감지",
        content = "…",
        riskLevel = NotificationAction.DANGER,
        area = "A-12",
        case = "화재",
        image = null,
        serial = "AMR-001",
        createAt = "2024-01-15 14:30",
    ),
    Report(
        id = 1L,
        title = "창고 A구역 화재 위험 감지",
        content = "…",
        riskLevel = NotificationAction.DANGER,
        area = "A-12",
        case = "화재",
        image = null,
        serial = "AMR-001",
        createAt = "2024-01-15 14:30",
    ),
    Report(
        id = 1L,
        title = "창고 A구역 화재 위험 감지",
        content = "…",
        riskLevel = NotificationAction.DANGER,
        area = "A-12",
        case = "화재",
        image = null,
        serial = "AMR-001",
        createAt = "2024-01-15 14:30",
    ),    Report(
        id = 1L,
        title = "창고 A구역 화재 위험 감지",
        content = "…",
        riskLevel = NotificationAction.DANGER,
        area = "A-12",
        case = "화재",
        image = null,
        serial = "AMR-001",
        createAt = "2024-01-15 14:30",
    ),

)