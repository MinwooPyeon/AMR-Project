package com.android.ssamr.feature.dashboard

import androidx.compose.foundation.background
import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.*
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.res.painterResource
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.tooling.preview.Preview
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.android.ssamr.R
import com.android.ssamr.core.domain.model.DashboardAmr
import com.android.ssamr.core.domain.model.DashboardAmrStatus
import com.android.ssamr.ui.theme.SSAMRTheme

data class DashboardSummaryItem(
    val title: String,
    val count: Int,
    val color: Color,
    val iconResId: Int
)

@Composable
fun TopSummarySection(summaryItems: List<DashboardSummaryItem>) {
    Column(verticalArrangement = Arrangement.spacedBy(12.dp)) {
        summaryItems.chunked(2).forEach { rowItems ->
            Row(
                horizontalArrangement = Arrangement.spacedBy(12.dp),
                modifier = Modifier.fillMaxWidth()
            ) {
                rowItems.forEach { item ->
                    SummaryCard(
                        title = item.title,
                        count = item.count,
                        color = item.color,
                        iconResId = item.iconResId,
                        modifier = Modifier.weight(1f)
                    )
                }
            }
        }
    }
}

@Composable
private fun SummaryCard(
    title: String,
    count: Int,
    color: Color,
    iconResId: Int,
    modifier: Modifier = Modifier
) {
    Card(
        modifier = modifier.height(80.dp),
        colors = CardDefaults.cardColors(containerColor = Color.White)
    ) {
        Row(
            modifier = Modifier
                .fillMaxSize()
                .padding(16.dp),
            verticalAlignment = Alignment.CenterVertically
        ) {
            Box(
                modifier = Modifier
                    .size(32.dp)
                    .background(
                        color = color.copy(alpha = 0.15f),
                        shape = RoundedCornerShape(8.dp)
                    ),
                contentAlignment = Alignment.Center
            ) {
                Icon(
                    painter = painterResource(id = iconResId),
                    contentDescription = title,
                    tint = color,
                    modifier = Modifier.size(20.dp)
                )
            }

            Spacer(modifier = Modifier.width(12.dp))

            Column {
                Text(
                    text = "$count",
                    fontWeight = FontWeight.Bold,
                    fontSize = 20.sp,
                    color = color
                )
                Text(
                    text = title,
                    fontSize = 14.sp,
                    color = Color.Black
                )
            }
        }
    }
}

@Composable
fun FactoryMapSection(
    onExpandClick: () -> Unit
) {
    Card(
        shape = MaterialTheme.shapes.medium,
        colors = CardDefaults.cardColors(containerColor = Color.White),
        modifier = Modifier.fillMaxWidth()
    ) {
        Column(Modifier.padding(16.dp)) {
            Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.SpaceBetween) {
                Text("공장지도", fontWeight = FontWeight.Bold)
                TextButton(onClick = onExpandClick) {
                    Text("전체화면", fontSize = 12.sp, color = Color(0xFF4C65E2))
                }
            }
            Spacer(Modifier.height(8.dp))
            Box(
                modifier = Modifier
                    .fillMaxWidth()
                    .height(200.dp)
                    .background(Color.LightGray)
            ) {
                // 지도 이미지 삽입 예정
            }
        }
    }
}

@Composable
fun AmrStatusSection(
    amrs: List<DashboardAmr>,
    onClick: (Long) -> Unit,
    onViewAllClick: () -> Unit
) {
    Card(
        shape = MaterialTheme.shapes.medium,
        colors = CardDefaults.cardColors(containerColor = Color.White),
        modifier = Modifier.fillMaxWidth()
    ) {
        Column(Modifier.padding(16.dp)) {
            Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.SpaceBetween) {
                Text("AMR 현황", fontWeight = FontWeight.Bold)
                TextButton(onClick = onViewAllClick) {
                    Text("전체보기", fontSize = 12.sp, color = Color(0xFF4C65E2))
                }
            }

            Spacer(Modifier.height(8.dp))

            amrs.forEach { amr ->
                Row(
                    modifier = Modifier
                        .fillMaxWidth()
                        .background(Color(0xFFF4F6FA), shape = RoundedCornerShape(12.dp))
                        .clickable { onClick(amr.id) }
                        .padding(vertical = 12.dp, horizontal = 12.dp),
                    horizontalArrangement = Arrangement.SpaceBetween,
                    verticalAlignment = Alignment.CenterVertically
                ) {
                    Column {
                        Text(amr.name, fontWeight = FontWeight.SemiBold)
                        Text(amr.location, fontSize = 12.sp, color = Color.Gray)
                    }
                    Column(horizontalAlignment = Alignment.End) {
                        Text(amr.status.display, color = amr.status.color)
                    }
                }
                Spacer(modifier = Modifier.height(16.dp))
            }
        }
    }
}

@Preview(showBackground = true)
@Composable
fun PreviewTopSummarySection() {
    val summaryItems = listOf(
        DashboardSummaryItem("총 AMR", 10, Color(0xFF58A74B), R.drawable.ic_robot),
        DashboardSummaryItem("작동중", 4, Color(0xFF3556F2), R.drawable.ic_running),
        DashboardSummaryItem("충전중", 3, Color(0xFFF7B500), R.drawable.ic_charging),
        DashboardSummaryItem("점검중", 3, Color(0xFFF7575C), R.drawable.ic_checking)
    )
    SSAMRTheme {
        TopSummarySection(summaryItems)
    }
}

@Preview(showBackground = true)
@Composable
fun PreviewFactoryMapSection() {
    SSAMRTheme {
        FactoryMapSection(onExpandClick = {})
    }
}

@Preview(showBackground = true)
@Composable
fun PreviewAmrStatusSection() {
    val dummyList = listOf(
        DashboardAmr(1L, "AMR-001", DashboardAmrStatus.RUNNING, "Zone A", "운반 중"),
        DashboardAmr(2L, "AMR-002", DashboardAmrStatus.CHARGING, "Zone B", "충전 중"),
        DashboardAmr(3L, "AMR-003", DashboardAmrStatus.CHECK, "Zone C", "점검 중")
    )
    SSAMRTheme {
        AmrStatusSection(
            amrs = dummyList,
            onClick = {},
            onViewAllClick = {}
        )
    }
}
