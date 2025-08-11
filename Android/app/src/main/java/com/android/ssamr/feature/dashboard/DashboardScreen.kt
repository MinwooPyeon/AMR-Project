package com.android.ssamr.feature.dashboard

import androidx.compose.foundation.background
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.lazy.LazyColumn
import androidx.compose.runtime.Composable
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.tooling.preview.Preview
import androidx.compose.ui.unit.dp
import com.android.ssamr.core.domain.model.DashboardAmr
import com.android.ssamr.core.domain.model.DashboardAmrStatus
import com.android.ssamr.ui.theme.SSAMRTheme
import com.android.ssamr.R

@Composable
fun DashboardScreen(
    state: DashboardState = DashboardState(),
    sendIntent: (DashboardIntent) -> Unit = {}
) {
    LazyColumn(
        modifier = Modifier
            .fillMaxSize()
            .padding(horizontal = 16.dp, vertical = 12.dp),
        verticalArrangement = Arrangement.spacedBy(16.dp)
    ) {
        item {
            TopSummarySection(
                summaryItems = listOf(
                    DashboardSummaryItem("총 AMR", state.totalCount, Color(0xFF58A74B), R.drawable.ic_robot),
                    DashboardSummaryItem("작동중", state.runningCount, Color(0xFF3556F2), R.drawable.ic_running),
                    DashboardSummaryItem("충전중", state.chargingCount, Color(0xFFF7B500), R.drawable.ic_charging),
                    DashboardSummaryItem("점검중", state.checkingCount, Color(0xFFF7575C), R.drawable.ic_checking)
                )
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
    DashboardAmr(
        id = 1L,
        name = "AMR-001",
        status = DashboardAmrStatus.RUNNING,
        location = "A구역-라인1",
    ),
    DashboardAmr(
        id = 2L,
        name = "AMR-002",
        status = DashboardAmrStatus.CHARGING,
        location = "충전소-1번",
    ),
    DashboardAmr(
        id = 3L,
        name = "AMR-003",
        status = DashboardAmrStatus.CHECKING,
        location = "B구역-라인3",
    )
)
