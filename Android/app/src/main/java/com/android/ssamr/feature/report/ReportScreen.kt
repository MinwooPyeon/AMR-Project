package com.android.ssamr.feature.report

import android.os.Build
import androidx.annotation.RequiresApi
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.PaddingValues
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.lazy.LazyColumn
import androidx.compose.foundation.lazy.items
import androidx.compose.foundation.lazy.rememberLazyListState
import androidx.compose.material3.Surface
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

//@OptIn(ExperimentalFoundationApi::class)
@RequiresApi(Build.VERSION_CODES.O)
@Composable
fun ReportScreen(
    state: ReportState,
    sendIntent: (ReportIntent) -> Unit = {}
) {
    val listState = rememberLazyListState()

    LazyColumn(
        state = listState,
        modifier = Modifier.fillMaxSize(),
        contentPadding = PaddingValues(vertical = 12.dp),
        verticalArrangement = Arrangement.spacedBy(12.dp)
    ) {
        // ① 상단 통계 카드 묶음
        item {
            Column(
                verticalArrangement = Arrangement.spacedBy(12.dp),
                modifier = Modifier.padding(horizontal = 12.dp)
            ) {
                Row(horizontalArrangement = Arrangement.spacedBy(12.dp), modifier = Modifier.fillMaxWidth()) {
                    ReportStatCard(
                        count = state.startCardCounts[ReportCategory.ALL],
                        label = "총 알림",
                        countColor = Color.Black,
                        modifier = Modifier.weight(1f)
                    )
                }
                Row(horizontalArrangement = Arrangement.spacedBy(12.dp), modifier = Modifier.fillMaxWidth()) {
                    ReportStatCard(state.startCardCounts[ReportCategory.COLLAPSE], "적재 사고", Color.Blue, Modifier.weight(1f))
                    ReportStatCard(state.startCardCounts[ReportCategory.SMOKE], "공장 내 흡연", Color(0xFF388E3C), Modifier.weight(1f))
                }
                Row(horizontalArrangement = Arrangement.spacedBy(12.dp), modifier = Modifier.fillMaxWidth()) {
                    ReportStatCard(state.startCardCounts[ReportCategory.EQUIPMENT], "안전 장비 미착용", Color(0xFFD32F2F), Modifier.weight(1f))
                    ReportStatCard(state.startCardCounts[ReportCategory.DANGER], "인명사고", Color(0xFFF57C00), Modifier.weight(1f))
                }
            }
        }

        // ② 차트 섹션(카드로 감싸서 보기 좋게)
        item {
            ReportChartsSection(
                state = state,
                sendIntent = sendIntent,
                modifier = Modifier.padding(horizontal = 12.dp)
            )
        }

        // ③ 탭(원하면 stickyHeader로 고정)
        stickyHeader {
            Surface(color = Color(0xFFF4F9FF)) { // 배경 색상 필요 없으면 제거
                ReportCategoryTabRow(
                    state = state,
                    sendIntent = sendIntent
                )
            }
        }

        // ④ 리스트 아이템들 (여기가 실제 스크롤 영역)
        items(
            items = state.reportList,
            key = { it.id }  // 같은 id는 한 번만
        ) { report ->
            // 기존 ReportCardList 안에서 사용하던 카드 컴포저블을 직접 호출
            ReportCard(
                report = report,
                onClick = { sendIntent(ReportIntent.ClickReportCard(report.id)) }
            )
        }

        // (선택) 하단 여백
        item { Spacer(Modifier.height(24.dp)) }
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