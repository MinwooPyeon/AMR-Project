package com.android.ssamr.feature.dashboard

import androidx.compose.foundation.background
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.lazy.LazyColumn
import androidx.compose.runtime.Composable
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.tooling.preview.Preview
import androidx.compose.ui.unit.dp
import com.android.ssamr.ui.theme.SSAMRTheme

@Composable
fun DashboardScreen(
    state: DashboardState = DashboardState(),
    sendIntent: (DashboardIntent) -> Unit = {}
) {
    LazyColumn(
        modifier = Modifier
            .fillMaxSize()
            .background(Color(0xFFF4F6FA))
            .padding(horizontal = 16.dp, vertical = 12.dp),
        verticalArrangement = Arrangement.spacedBy(16.dp)
    ) {
        item {
            // TopSummarySection은 DashboardContent.kt에 정의됨
            TopSummarySection(
                total = state.totalCount,
                running = state.runningCount,
                charging = state.chargingCount,
                checking = state.checkingCount
            )
        }

        item {
            FactoryMapSection(
                onExpandClick = { sendIntent(DashboardIntent.ClickMapExpand) }
            )
        }

        item {
            AmrStatusSection(
                amrs = state.amrList,
                onClick = { id -> sendIntent(DashboardIntent.ClickAmrItem(id)) },
                onViewAllClick = { sendIntent(DashboardIntent.ClickViewAllAmr) }
            )
        }
    }
}

@Preview(showBackground = true)
@Composable
fun DashboardScreenPreview() {
    SSAMRTheme {
        DashboardScreen(
            state = DashboardState(
                amrList = sampleDashboardAmrs,
                totalCount = 4,
                runningCount = 2,
                chargingCount = 1,
                checkingCount = 1
            )
        )
    }
}

// 샘플 데이터
val sampleDashboardAmrs = listOf(
    DashboardAmrUiModel(
        id = 1L,
        name = "AMR-001",
        status = DashboardAmrStatus.RUNNING,
        location = "A구역-라인1",
        job = "화물 운반 중"
    ),
    DashboardAmrUiModel(
        id = 2L,
        name = "AMR-002",
        status = DashboardAmrStatus.CHARGING,
        location = "충전소-1번",
        job = "충전 중"
    ),
    DashboardAmrUiModel(
        id = 3L,
        name = "AMR-003",
        status = DashboardAmrStatus.CHECK,
        location = "B구역-라인3",
        job = "점검 중"
    )
)
