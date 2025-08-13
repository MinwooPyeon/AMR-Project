// ReportContent.kt
package com.android.ssamr.feature.report

import android.os.Build
import androidx.annotation.RequiresApi
import androidx.compose.animation.AnimatedVisibility
import androidx.compose.animation.expandVertically
import androidx.compose.animation.fadeIn
import androidx.compose.animation.fadeOut
import androidx.compose.animation.shrinkVertically
import androidx.compose.foundation.BorderStroke
import androidx.compose.foundation.border
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.*
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.Shape
import androidx.compose.ui.layout.ContentScale
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.style.TextOverflow
import androidx.compose.ui.tooling.preview.Preview
import androidx.compose.ui.unit.Dp
import androidx.compose.ui.unit.dp
import coil.compose.AsyncImage
import coil.request.ImageRequest
import com.android.ssamr.BuildConfig
import com.android.ssamr.R
import com.android.ssamr.core.domain.model.NotificationAction
import com.android.ssamr.core.domain.model.Report
import com.android.ssamr.core.common.time.formatMonthDayTimeKorean
import com.android.ssamr.ui.theme.SSAMRTheme
import java.time.ZoneId

/* =========================================================
 * 공용 카드/칩 (Content 전용)
 * ========================================================= */

@Composable
fun SectionCard(
    title: String? = null,
    modifier: Modifier = Modifier,
    containerColor: Color = Color.White,
    corner: Dp = 20.dp,
    contentPadding: PaddingValues = PaddingValues(16.dp),
    topRightAction: (@Composable RowScope.() -> Unit)? = null,
    content: @Composable ColumnScope.() -> Unit
) {
    Card(
        modifier = modifier,
        colors = CardDefaults.cardColors(containerColor = containerColor),
        shape = RoundedCornerShape(corner),
        elevation = CardDefaults.cardElevation(defaultElevation = 0.dp)
    ) {
        Column(Modifier.fillMaxWidth().padding(contentPadding)) {
            if (!title.isNullOrBlank() || topRightAction != null) {
                Row(verticalAlignment = Alignment.CenterVertically) {
                    if (!title.isNullOrBlank()) {
                        Text(
                            text = title!!,
                            style = MaterialTheme.typography.titleLarge,
                            fontWeight = FontWeight.SemiBold,
                            color = Color(0xFF1B1E27),
                            modifier = Modifier.weight(1f),
                            maxLines = 1,
                            overflow = TextOverflow.Ellipsis
                        )
                    } else {
                        Spacer(Modifier.weight(1f))
                    }
                    topRightAction?.invoke(this)
                }
                Spacer(Modifier.height(12.dp))
            }
            content()
        }
    }
}

@Composable
fun SoftChip(
    text: String,
    bg: Color,
    fg: Color,
    modifier: Modifier = Modifier,
    shape: Shape = RoundedCornerShape(999.dp)
) {
    Surface(color = bg, shape = shape, modifier = modifier) {
        Text(
            text = text,
            color = fg,
            style = MaterialTheme.typography.labelMedium,
            modifier = Modifier.padding(horizontal = 10.dp, vertical = 6.dp)
        )
    }
}

/* =========================================================
 * 상세 화면용 Content 컴포넌트들 (Screen에서 호출)
 * ========================================================= */

@RequiresApi(Build.VERSION_CODES.O)
@Composable
fun ReportDetailHeaderCard(
    report: Report,
    modifier: Modifier = Modifier
) {
    SectionCard(
        title = report.title,
        modifier = modifier
    ) {
        // 태그 칩들
        Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
            val (caseLabel, caseBg, caseFg) = caseStyle(report.case)
            SoftChip(caseLabel, caseBg, caseFg)

            val (riskLabel, riskBg, riskFg) = riskStyle(report.riskLevel)
            SoftChip(riskLabel, riskBg, riskFg)
        }

        Spacer(Modifier.height(12.dp))

        // 2열 정보 그리드
        InfoGrid(
            leftTitle = "발생 시간",
            leftValue = safeKoreanTime(report.createAt),
            rightTitle = "발생 위치",
            rightValue = report.area
        )
        Spacer(Modifier.height(8.dp))
        InfoGrid(
            leftTitle = "관련 AMR",
            leftValue = report.serial,
            rightTitle = "영향도",
            rightValue = riskLabel(report.riskLevel)
        )
    }
}



@RequiresApi(Build.VERSION_CODES.O)
@Composable
fun ReportWebcamImageContent(
    report: Report,
    expanded: Boolean,             // ✅ 화면 상태에서 내려받음 (state.isImageExpanded)
    onToggle: () -> Unit,          // ✅ 버튼 눌렀을 때 토글 인텐트 보내기
    modifier: Modifier = Modifier
) {
    SectionCard(
        title = "현장 웹캠 이미지",
        modifier = modifier,
        topRightAction = {
            val shape = RoundedCornerShape(14.dp)
            Button(
                onClick = onToggle,
                shape = shape,
                colors = ButtonDefaults.buttonColors(
                    containerColor = Color(0xFF4C65E2),
                    contentColor = Color.White
                ),
                contentPadding = PaddingValues(horizontal = 16.dp, vertical = 8.dp),
                modifier = Modifier
                    .heightIn(min = 36.dp)) {
                Text(if (expanded) "닫기" else "보기")
            }
        }
    ) {
        AnimatedVisibility(
            visible = expanded,
            enter = expandVertically() + fadeIn(),
            exit = shrinkVertically() + fadeOut()
        ) {
            Column {
                val shape = RoundedCornerShape(16.dp)

                if (report.image.isNullOrBlank()) {
                    Box(
                        modifier = Modifier
                            .fillMaxWidth()
                            .height(180.dp)
                            .clip(shape),
                        contentAlignment = Alignment.Center
                    ) { Text("이미지를 불러올 수 없어요", color = Color.Gray) }
                } else {
                    AsyncImage(
                        model = ImageRequest.Builder(LocalContext.current)
                            .data("${BuildConfig.BASE_URL.dropLast(1)}${report.image}")
                            .crossfade(true)
                            .placeholder(com.android.ssamr.R.drawable.ic_image_placeholder) // 선택
                            .error(R.drawable.ic_image_placeholder)       // 선택
                            .build(),
                        contentDescription = "현장 이미지",
                        contentScale = ContentScale.Crop,
                        modifier = Modifier
                            .fillMaxWidth()
                            .height(300.dp)
                            .clip(shape)
                    )
                }

                Spacer(Modifier.height(12.dp))
                Text(
                    text = "${safeKoreanTime(report.createAt)}  촬영 - ${report.area}",
                    style = MaterialTheme.typography.bodyMedium,
                    color = Color(0xFF7C8899)
                )
            }
        }
    }
}

/* ---------- 공용 소형 조각 ---------- */

@Composable
fun InfoGrid(
    leftTitle: String,
    leftValue: String,
    rightTitle: String,
    rightValue: String
) {
    Row(Modifier.fillMaxWidth()) {
        InfoColumn(leftTitle, leftValue, Modifier.weight(1f))
        InfoColumn(rightTitle, rightValue, Modifier.weight(1f))
    }
}

@Composable
fun InfoColumn(
    title: String,
    value: String,
    modifier: Modifier = Modifier
) {
    Column(modifier) {
        Text(title, style = MaterialTheme.typography.bodyMedium, color = Color(0xFF7C8899))
        Spacer(Modifier.height(4.dp))
        Text(
            value,
            style = MaterialTheme.typography.titleMedium,
            color = Color(0xFF1B1E27),
            fontWeight = FontWeight.SemiBold
        )
    }
}

/* ---------- 스타일/레이블 유틸 ---------- */

private fun riskLabel(level: NotificationAction): String = when (level) {
    NotificationAction.DANGER -> "위험"
    NotificationAction.WARNING -> "경고"
    NotificationAction.INFORMATION -> "정보"
}
private fun riskStyle(level: NotificationAction): Triple<String, Color, Color> = when (level) {
    NotificationAction.DANGER -> Triple("위험", Color(0xFFFFE0E0), Color(0xFFDC2626))
    NotificationAction.WARNING -> Triple("경고", Color(0xFFFFF1D6), Color(0xFFF57C00))
    NotificationAction.INFORMATION -> Triple("정보", Color(0xFFE6EEFF), Color(0xFF4C65E2))
}

private fun caseStyle(case: String): Triple<String, Color, Color> =
    when (case.uppercase()) {
        "SMOKE" -> Triple("화재", Color(0xFFD8B5FF), Color(0xFF9C44EC))
        "EQUIPMENT"     -> Triple("안전 장비 미착용", Color(0xFFA6C4D2), Color(0xFF5184A0))
        "COLLAPSE"      -> Triple("적재사고", Color(0xFFFFE9B0), Color(0xFFB4812E))
        "DANGER" -> Triple("안전사고", Color(0xFFFFD1D1), Color(0xFFDC2626))
        else            -> Triple(case, Color(0xFFEFEFEF), Color(0xFF444444))
    }

@RequiresApi(Build.VERSION_CODES.O)
private fun safeKoreanTime(raw: String): String =
    runCatching { formatMonthDayTimeKorean(raw, ZoneId.systemDefault()) }
        .getOrElse { raw }

@RequiresApi(Build.VERSION_CODES.O)
@Preview(showBackground = true)
@Composable
fun ReportDetailHeaderCardPreivew() {
    SSAMRTheme {
        ReportDetailHeaderCard(
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
    }
}

@Preview(showBackground = true)
@Composable
fun ReportWebcamImageContentPreview() {
    SSAMRTheme {
        ReportWebcamImageContent(
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
            false,
            {}
        )
    }
}