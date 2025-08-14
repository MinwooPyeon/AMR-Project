package com.android.ssamr.feature.reportDetail

import android.os.Build
import androidx.annotation.RequiresApi
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.PaddingValues
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.lazy.LazyColumn
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.saveable.rememberSaveable
import androidx.compose.runtime.setValue
import androidx.compose.ui.Modifier
import androidx.compose.ui.tooling.preview.Preview
import androidx.compose.ui.unit.dp
import com.android.ssamr.core.domain.model.NotificationAction
import com.android.ssamr.core.domain.model.Report
import com.android.ssamr.feature.report.ReportDetailHeaderCard
import com.android.ssamr.feature.report.ReportWebcamImageContent
import com.android.ssamr.ui.theme.SSAMRTheme

@RequiresApi(Build.VERSION_CODES.O)
@Composable
fun ReportDetailScreen(
    state: ReportDetailState,
    sendIntent: (ReportDetailIntent) -> Unit = {},
    modifier: Modifier = Modifier
) {
    var expanded by rememberSaveable { mutableStateOf(false) }
    val report = state.report

    LazyColumn(
        modifier = modifier.fillMaxSize(),
        contentPadding = PaddingValues(12.dp),
        verticalArrangement = Arrangement.spacedBy(12.dp)
    ) {
        if (report != null) {
            item {
                ReportDetailHeaderCard(report = report, modifier = Modifier.padding(horizontal = 12.dp))
            }
            item {
                ReportWebcamImageContent(
                    report = report,
                    expanded = expanded,
                    onToggle = { expanded = !expanded },
                    modifier = Modifier.padding(horizontal = 12.dp)
                )
            }
            item { Spacer(Modifier.height(8.dp)) }
        } else {
            // (옵션) 로딩/빈 상태 UI
            item { Spacer(Modifier.height(1.dp)) }
        }
    }
}

@Preview(showBackground = true)
@Composable
fun ReportDetailScreenPreview() {
    SSAMRTheme {
        ReportDetailScreen(
            state = ReportDetailState(
                report = Report(
                    id = 1L,
                    title = "창고 A구역 화재 위험 감지",
                    content = "…",
                    riskLevel = NotificationAction.DANGER,
                    area = "A-12",
                    case = "SMOKE",
                    image = null,
                    serial = "AMR-001",
                    createAt = "2024-01-15 14:30",
                ),
            )
        )
    }
}