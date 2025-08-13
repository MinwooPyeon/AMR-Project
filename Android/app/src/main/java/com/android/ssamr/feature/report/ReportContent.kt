package com.android.ssamr.feature.report

import android.os.Build
import androidx.annotation.RequiresApi
import androidx.compose.animation.core.Spring
import androidx.compose.animation.core.spring
import androidx.compose.animation.core.tween
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
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.SolidColor
import androidx.compose.ui.graphics.drawscope.DrawStyle
import androidx.compose.ui.graphics.drawscope.Fill
import androidx.compose.ui.res.painterResource
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.style.TextOverflow
import androidx.compose.ui.tooling.preview.Preview
import androidx.compose.ui.unit.dp
import com.android.ssamr.R
import com.android.ssamr.core.common.time.formatMonthDayTimeKorean
import com.android.ssamr.core.domain.model.NotificationAction
import com.android.ssamr.core.domain.model.Report
import com.android.ssamr.core.domain.model.ReportAction
import com.android.ssamr.core.domain.model.ReportCategory
import com.android.ssamr.ui.theme.SSAMRTheme
import ir.ehsannarmani.compose_charts.ColumnChart
import ir.ehsannarmani.compose_charts.PieChart
import ir.ehsannarmani.compose_charts.RowChart
import ir.ehsannarmani.compose_charts.models.BarProperties
import ir.ehsannarmani.compose_charts.models.Bars
import ir.ehsannarmani.compose_charts.models.Pie
import java.time.LocalDate
import java.time.LocalDateTime
import java.time.ZoneId
import java.time.format.DateTimeFormatter
import kotlin.collections.listOf

@Composable
fun ReportStatCard(
    count: Int?,
    label: String,
    countColor: Color,
    modifier: Modifier = Modifier
) {
    Card(
        shape = RoundedCornerShape(16.dp),
        colors = CardDefaults.cardColors(containerColor = Color.White),
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
    val action = try {
        ReportAction.valueOf(report.case) // 문자열 -> enum 변환
    } catch (e: IllegalArgumentException) {
        null
    }

    val primaryTagColor = when (action) {
        ReportAction.COLLAPSE  -> Color(0xFFFFE9B0) // 연노랑
        ReportAction.SMOKE     -> Color(0xFFF3E8FF) // 연분홍
        ReportAction.EQUIPMENT -> Color(0xFFD7E6FF) // 연파랑
        ReportAction.DANGER    -> Color(0xFFFFD1D1) // 연분홍
        null                   -> Color.Gray
    }

    val primaryTextColor = when (action) {
        ReportAction.COLLAPSE  -> Color(0xFFB4812E)
        ReportAction.SMOKE     -> Color(0xFF9C44EC)
        ReportAction.EQUIPMENT -> Color(0xFF5184A0)
        ReportAction.DANGER    -> Color(0xFFDC2626)
        null                   -> Color.DarkGray
    }

    val primaryTagText = when (action) {
        ReportAction.COLLAPSE  -> "적재불안정"
        ReportAction.SMOKE     -> "흡연"
        ReportAction.EQUIPMENT -> "장비고장"
        ReportAction.DANGER    -> "안전사고"
        null                   -> "기타"
    }

    Card(
        modifier = Modifier
            .fillMaxWidth()
            .padding(horizontal = 12.dp)
            .clickable { onClick(report.id) },
        shape = RoundedCornerShape(20.dp),
        colors = CardDefaults.cardColors(containerColor = Color.White),
        elevation = CardDefaults.cardElevation(defaultElevation = 1.5.dp)
    ) {
        Column(Modifier.padding(16.dp)) {

            // 상단: 태그 칩들 + 해결여부
            Row(verticalAlignment = Alignment.CenterVertically) {
                TagChip(label = primaryTagText, bg = primaryTagColor, fg = primaryTextColor)

                Spacer(Modifier.width(6.dp))
                // 스샷처럼 추가 칩들
                TagChip(label = "위험", bg = Color(0xFFF2F4F6), fg = Color(0xFF6B7684))

            }

            Spacer(Modifier.height(10.dp))

            // 제목
            Text(
                text = report.title,
                style = MaterialTheme.typography.titleLarge,
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
                    style = MaterialTheme.typography.bodyLarge,
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
                    style = MaterialTheme.typography.bodyLarge,
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
fun ReportChartsSection(
    state: ReportState,
    sendIntent: (ReportIntent) -> Unit,
    modifier: Modifier = Modifier
) {
    Column(modifier = modifier.fillMaxWidth()) {
        Card(
            modifier = Modifier
                .fillMaxWidth()
                .padding(horizontal = 12.dp, vertical = 6.dp),
            shape = RoundedCornerShape(20.dp),
            colors = CardDefaults.cardColors(containerColor = Color.White),
        ) {
            Column(
                modifier = Modifier.padding(16.dp)
            ) {
                Text(
                    text = "이벤트 발생 현황",
                    style = MaterialTheme.typography.titleLarge,
                    fontWeight = FontWeight.SemiBold,
                )
                Spacer(Modifier.height(16.dp))
                CategoryDonutChart(
                    counts = state.startCardCounts,
                    onSliceClick = { category ->
                        sendIntent(ReportIntent.ClickReportCategory(category))
                    }
                )

                Spacer(Modifier.height(32.dp))

                RiskRowChart(
                    danger = state.fullReportList.count { it.riskLevel == NotificationAction.DANGER },
                    warning = state.fullReportList.count { it.riskLevel == NotificationAction.WARNING },
                    info = state.fullReportList.count { it.riskLevel == NotificationAction.INFORMATION }
                )

                Spacer(Modifier.height(32.dp))

                WeeklyColumnChart(dateStrings = state.fullReportList.map { it.createAt })
            }
        }

    }
}
@Composable
fun CategoryDonutChart(
    counts: Map<ReportCategory, Int>,
    onSliceClick: (ReportCategory) -> Unit,
) {
    val labelToCategory = ReportCategory.values().associateBy { it.label }

    val pieData by remember(counts) {
        mutableStateOf(
            listOf(
                Pie(ReportCategory.COLLAPSE.label, (counts[ReportCategory.COLLAPSE] ?: 0).toDouble(),
                    color = Color(0xFFB4812E), selectedColor = Color(0xFFE9C789)),
                Pie(ReportCategory.SMOKE.label, (counts[ReportCategory.SMOKE] ?: 0).toDouble(),
                    color = Color(0xFF9C44EC), selectedColor = Color(0xFFD8B5FF)),
                Pie(ReportCategory.EQUIPMENT.label, (counts[ReportCategory.EQUIPMENT] ?: 0).toDouble(),
                    color = Color(0xFF5184A0), selectedColor = Color(0xFFA6C4D2)),
                Pie(ReportCategory.DANGER.label, (counts[ReportCategory.DANGER] ?: 0).toDouble(),
                    color = Color(0xFFDC2626), selectedColor = Color(0xFFFF9C9C)),
            )
        )
    }

    PieChart(
        modifier = Modifier
            .fillMaxWidth()
            .height(200.dp)
            .padding(horizontal = 12.dp),
        data = pieData,
        style = Pie.Style.Stroke(width = 28.dp), // ✅ Float
        spaceDegree = 6f,
        selectedPaddingDegree = 4f,
        onPieClick = { pie ->
            labelToCategory[pie.label]?.let { onSliceClick(it) } // ✅ 콜백 실제 호출
        },
        scaleAnimEnterSpec = spring(
            dampingRatio = Spring.DampingRatioMediumBouncy,
            stiffness = Spring.StiffnessLow
        ),
        colorAnimEnterSpec = tween(300),
        colorAnimExitSpec = tween(300),
        scaleAnimExitSpec = tween(300),
        spaceDegreeAnimExitSpec = tween(300),
    )
}

@Composable
private fun RiskRowChart(
    danger: Int,
    warning: Int,
    info: Int
) {
    val bars = listOf(
        Bars(
            label = "위험도",
            values = listOf(
                Bars.Data(label = "위험", value = danger.toDouble(), color = SolidColor(Color(0xFFDC2626))),
                Bars.Data(label = "경고", value = warning.toDouble(), color = SolidColor(Color(0xFFF57C00))),
                Bars.Data(label = "정보", value = info.toDouble(), color = SolidColor(Color(0xFF4C65E2))),
            )
        )
    )

    RowChart(
        modifier = Modifier
            .fillMaxWidth()
            .height(180.dp)
            .padding(horizontal = 12.dp),
        data = bars,
        barProperties = BarProperties(
            thickness = 22.dp,                                   // ✅ was strokeWidth
            spacing = 6.dp,
            cornerRadius = Bars.Data.Radius.Circular(8.dp),      // ✅ was radius=Rectangle(...)
            style = ir.ehsannarmani.compose_charts.models.DrawStyle.Fill                               // 선택사항
        ),
    )
}


@RequiresApi(Build.VERSION_CODES.O)
@Composable
private fun WeeklyColumnChart(
    dateStrings: List<String>,
    days: Int = 7,
    alignToLatestEvent: Boolean = true,      // ✅ 핵심 옵션
) {
    val zone = remember { ZoneId.systemDefault() }

    // 1) 문자열 → LocalDate 변환 (네 유틸 활용)
    val dates: List<LocalDate> = remember(dateStrings) {
        dateStrings.mapNotNull { com.android.ssamr.core.common.time.parseToLocalDateOrNull(it, zone) }
    }

    // 2) 기준일: 데이터가 있으면 가장 최근 날짜, 없으면 오늘
    val anchor: LocalDate = remember(dates) {
        if (alignToLatestEvent && dates.isNotEmpty()) dates.maxOrNull()!!
        else LocalDate.now(zone)
    }

    // 3) 윈도우(최근 N일)
    val windowDays: List<LocalDate> = remember(anchor, days) {
        (0 until days).map { anchor.minusDays((days - 1 - it).toLong()) }
    }

    // 4) 카운팅
    val counts: Map<LocalDate, Int> = remember(dates, windowDays) {
        val base = windowDays.associateWith { 0 }.toMutableMap()
        for (d in dates) if (d in base) base[d] = base.getValue(d) + 1
        base.toMap()
    }

    val total = counts.values.sum()

    // 5) Bars 데이터로 변환
    val bars = counts.map { (day, cnt) ->
        Bars(
            label = "${day.monthValue}/${day.dayOfMonth}",
            values = listOf(Bars.Data(value = cnt.toDouble(), color = SolidColor(Color(0xFF4C65E2))))
        )
    }

    Column {
        if (total == 0) {
            Text(
                text = "선택 기간에 이벤트가 없습니다.",
                style = MaterialTheme.typography.bodyMedium,
                color = Color.Gray,
                modifier = Modifier.padding(start = 12.dp, bottom = 8.dp)
            )
        }

        ColumnChart(
            modifier = Modifier
                .fillMaxWidth()
                .height(200.dp)
                .padding(horizontal = 12.dp),
            data = bars,
            barProperties = BarProperties(
                thickness = 18.dp,
                spacing = 4.dp,
                cornerRadius = Bars.Data.Radius.Circular(8.dp)
            )
        )
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
                riskLevel = NotificationAction.DANGER,
                area = "A-12",
                case = "SMOKE",
                image = null,
                serial = "AMR-001",
                createAt = "2024-01-15 14:30",
            ),
            onClick = { id -> /* navigate to detail */ }
        )
    }
}