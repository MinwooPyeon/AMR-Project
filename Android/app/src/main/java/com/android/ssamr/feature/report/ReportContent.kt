package com.android.ssamr.feature.report

import android.os.Build
import androidx.annotation.RequiresApi
import androidx.compose.foundation.background
import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.PaddingValues
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.layout.widthIn
import androidx.compose.foundation.lazy.LazyColumn
import androidx.compose.foundation.lazy.LazyRow
import androidx.compose.foundation.lazy.items
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.Card
import androidx.compose.material3.CardDefaults
import androidx.compose.material3.Icon
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Text
import androidx.compose.material3.TextButton
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.res.painterResource
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.style.TextOverflow
import androidx.compose.ui.tooling.preview.Preview
import androidx.compose.ui.unit.dp
import com.android.ssamr.R
import com.android.ssamr.core.common.time.formatMonthDayTimeKorean
import com.android.ssamr.core.domain.model.Report
import com.android.ssamr.core.domain.model.ReportAction
import com.android.ssamr.core.domain.model.ReportCategory
import com.android.ssamr.ui.theme.SSAMRTheme

@Composable
fun ReportStatCard(
    count: Int?,
    label: String,
    countColor: Color,
    modifier: Modifier = Modifier
) {
    Card(
        shape = RoundedCornerShape(16.dp),
        colors = CardDefaults.cardColors(containerColor = MaterialTheme.colorScheme.surface),
        modifier = modifier
    ) {
        Column(
            modifier = Modifier
                .padding(vertical = 16.dp)
                .fillMaxWidth(),
            horizontalAlignment = Alignment.CenterHorizontally
        ) {
            Text(
                text = count.toString(),
                style = MaterialTheme.typography.headlineSmall.copy(
                    fontWeight = FontWeight.Bold,
                    color = countColor
                )
            )
            Spacer(Modifier.height(4.dp))
            Text(
                text = label,
                style = MaterialTheme.typography.bodyMedium.copy(
                    color = MaterialTheme.colorScheme.onSurfaceVariant
                )
            )
        }
    }
}

@Composable
fun ReportCategoryTabRow(
    state: ReportState,
    sendIntent: (ReportIntent) -> Unit
) {
    val categories = ReportCategory.values()
    Box(
        Modifier
            .fillMaxWidth()
            .padding(vertical = 8.dp)
    ) {
        LazyRow(
            modifier = Modifier.fillMaxWidth(),
            verticalAlignment = Alignment.CenterVertically,
            contentPadding = PaddingValues(horizontal = 12.dp),
            horizontalArrangement = Arrangement.spacedBy(8.dp)
        ) {
            items(categories) { category ->
                val selected = category == state.selectedCategory
                TextButton(
                    onClick = { sendIntent(ReportIntent.ClickReportCategory(category)) },
                    modifier = Modifier
                        .height(60.dp)
                        .widthIn(min = 80.dp)
                ) {
                    Text(
                        text = "${category.label}",
                        color = if (selected) Color.White else Color(0xFF828282),
                        maxLines = 1,
                        overflow = TextOverflow.Ellipsis,
                        modifier = Modifier
                            .background(
                                if (selected) Color(0xFF4C65E2) else Color(0xFFF3F4F6),
                                shape = RoundedCornerShape(20.dp)
                            )
                            .padding(horizontal = 20.dp, vertical = 12.dp)
                    )
                }
            }
        }
    }
}

@RequiresApi(Build.VERSION_CODES.O)
@Composable
fun ReportCard(
    report: Report,
    onClick: (Long) -> Unit
) {
    val primaryTagColor = when (report.riskLevel) {
        ReportAction.COLLAPSE  -> Color(0xFFFFE9B0) // 연노랑
        ReportAction.SMOKE     -> Color(0xFFFFD1D1) // 연분홍
        ReportAction.EQUIPMENT -> Color(0xFFD7E6FF) // 연파랑
        ReportAction.DANGER    -> Color(0xFFFFD1D1) // 연분홍
    }
    val primaryTagText = when (report.riskLevel) {
        ReportAction.COLLAPSE  -> "적재불안정"
        ReportAction.SMOKE     -> "흡연"
        ReportAction.EQUIPMENT -> "장비고장"
        ReportAction.DANGER    -> "안전사고"
    }

    Card(
        modifier = Modifier
            .fillMaxWidth()
            .padding(horizontal = 12.dp, vertical = 6.dp)
            .clickable { onClick(report.id) },
        shape = RoundedCornerShape(20.dp),
        colors = CardDefaults.cardColors(containerColor = Color.White),
        elevation = CardDefaults.cardElevation(defaultElevation = 1.5.dp)
    ) {
        Column(Modifier.padding(16.dp)) {

            // 상단: 태그 칩들 + 해결여부
            Row(verticalAlignment = Alignment.CenterVertically) {
                TagChip(label = primaryTagText, bg = primaryTagColor, fg = Color(0xFF5B5B5B))

                Spacer(Modifier.width(6.dp))
                // 스샷처럼 추가 칩들
                TagChip(label = "위험", bg = Color(0xFFF2F4F6), fg = Color(0xFF6B7684))

            }

            Spacer(Modifier.height(10.dp))

            // 제목
            Text(
                text = report.title,
                style = MaterialTheme.typography.titleMedium,
                fontWeight = FontWeight.ExtraBold,
                color = Color(0xFF111322)
            )

            Spacer(Modifier.height(10.dp))

            // 위치 / AMR 시리얼
            Row(verticalAlignment = Alignment.CenterVertically) {
                Icon(
                    painter = painterResource(id = R.drawable.current_location),
                    contentDescription = "위치",
                    tint = Color(0xFF98A2B3),
                    modifier = Modifier.size(16.dp)
                )
                Spacer(Modifier.width(6.dp))
                Text(
                    text = "${report.area} 구역",
                    style = MaterialTheme.typography.bodyMedium,
                    color = Color(0xFF4E5968)
                )

                Spacer(Modifier.width(14.dp))

                Icon(
                    painter = painterResource(id = R.drawable.ic_robot), // 로봇/기기 아이콘 대체 리소스
                    contentDescription = "AMR",
                    tint = Color(0xFF98A2B3),
                    modifier = Modifier.size(16.dp)
                )
                Spacer(Modifier.width(6.dp))
                Text(
                    text = report.serial,
                    style = MaterialTheme.typography.bodyMedium,
                    color = Color(0xFF4E5968)
                )
            }

            Spacer(Modifier.height(16.dp))

            // 하단: 날짜 + 우측 화살표
            Row(verticalAlignment = Alignment.CenterVertically) {
                Text(
                    text = formatMonthDayTimeKorean(report.createAt),
                    style = MaterialTheme.typography.bodySmall,
                    color = Color(0xFF8B95A1)
                )
                Spacer(Modifier.weight(1f))
                Icon(
                    painter = painterResource(R.drawable.ic_right_arrow),
                    contentDescription = "상세",
                    tint = Color(0xFFB0B8C1),
                    modifier = Modifier.size(18.dp)
                )
            }
        }
    }
}

@RequiresApi(Build.VERSION_CODES.O)
@Composable
fun ReportCardList(
    reports: List<Report>,
    onCardClick: (Long) -> Unit
) {
    LazyColumn(
        modifier = Modifier
            .padding(horizontal = 8.dp)
    ) {
        items(reports) {report ->
            ReportCard(report = report,
                onClick = onCardClick
            )
        }
    }
}

@Composable
private fun TagChip(
    label: String,
    bg: Color,
    fg: Color,
) {
    Box(
        modifier = Modifier
            .clip(RoundedCornerShape(999.dp))
            .background(bg)
            .padding(horizontal = 10.dp, vertical = 5.dp)
    ) {
        Text(
            text = label,
            style = MaterialTheme.typography.labelMedium,
            color = fg
        )
    }
}

@Preview(showBackground = true)
@Composable
fun ReportStatsGrid() {
    Column(verticalArrangement = Arrangement.spacedBy(12.dp)) {
        Row(horizontalArrangement = Arrangement.spacedBy(12.dp), modifier = Modifier.fillMaxWidth()) {
            ReportStatCard(count = 5, label = "총 이벤트", countColor = Color.Black, modifier = Modifier.weight(1f))
            ReportStatCard(count = 4, label = "해결 완료", countColor = Color(0xFF388E3C), modifier = Modifier.weight(1f))
        }
        Row(horizontalArrangement = Arrangement.spacedBy(12.dp), modifier = Modifier.fillMaxWidth()) {
            ReportStatCard(count = 2, label = "위험 이벤트", countColor = Color(0xFFD32F2F), modifier = Modifier.weight(1f))
            ReportStatCard(count = 2, label = "경고 이벤트", countColor = Color(0xFFF57C00), modifier = Modifier.weight(1f))
        }
    }
}

@Preview(showBackground = true)
@Composable
fun ReportCategoryTabRowPreview() {
    SSAMRTheme {
        ReportCategoryTabRow(
            ReportState()
        ) {}

    }
}

@RequiresApi(Build.VERSION_CODES.O)
@Preview(showBackground = true)
@Composable
fun ReportCardPreview() {
    SSAMRTheme {
        ReportCard(
            report = Report(
                id = 1L,
                title = "창고 A구역 화재 위험 감지",
                content = "…",
                riskLevel = ReportAction.DANGER,
                area = "A-12",
                case = "화재",
                image = null,
                serial = "AMR-001",
                createAt = "2024-01-15 14:30",
            ),
            onClick = { id -> /* navigate to detail */ }
        )
    }
}