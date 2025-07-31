package com.android.ssamr.feature.amr

import androidx.compose.foundation.background
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
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.Card
import androidx.compose.material3.CardDefaults
import androidx.compose.material3.HorizontalDivider
import androidx.compose.material3.Icon
import androidx.compose.material3.LinearProgressIndicator
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Text
import androidx.compose.material3.TextButton
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.rotate
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.vector.ImageVector
import androidx.compose.ui.res.vectorResource
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.style.TextOverflow
import androidx.compose.ui.tooling.preview.Preview
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.android.ssamr.R
import com.android.ssamr.ui.theme.SSAMRTheme

@Composable
fun AmrCardList(
    amrs: List<AmrUiModel>,
    onAmrCardClick: (Long) -> Unit
) {
    LazyColumn(
        modifier = Modifier
            .padding(horizontal = 8.dp)
    ) {
        items(amrs) { amr ->
            AmrCard(amr = amr, onAmrCardClick)
        }
    }
}

@Composable
fun AmrCategoryTabRow(
    state: AmrState,
    sendIntent: (AmrIntent) -> Unit
) {
    val categories = AmrCategory.values()
    Box(
        Modifier
            .fillMaxWidth()
            .background(Color(0xFFF4F6FA))
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
                    onClick = { sendIntent(AmrIntent.ClickAmrCategory(category)) },
                    modifier = Modifier
                        .height(60.dp)
                        .widthIn(min = 80.dp)
                ) {
                    Text(
                        text = "${category.label} (${state.categoryCounts[category] ?: 0})",
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

@Composable
fun AmrCard(
    amr: AmrUiModel,
    onAmrCardClick: (Long) -> Unit
) {
    val statusColor = when (amr.status) {
        AmrStatus.RUNNING -> Color(0xFF4CAF50)
        AmrStatus.CHARGING -> Color(0xFFF7B500)
        AmrStatus.CHECK -> Color(0xFFF7575C)
        else -> Color.Gray
    }
    Card(
        modifier = Modifier
            .fillMaxWidth()
            .padding(vertical = 4.dp),
        shape = RoundedCornerShape(16.dp),
        colors = CardDefaults.cardColors(containerColor = Color.White),
        elevation = CardDefaults.cardElevation(defaultElevation = 2.dp),
        onClick = { onAmrCardClick(amr.id) }
    ) {
        Column(Modifier.padding(16.dp)) {
            Row(verticalAlignment = Alignment.CenterVertically) {
                Box(
                    modifier = Modifier
                        .size(16.dp)
                        .background(statusColor, CircleShape)
                )
                Spacer(Modifier.width(8.dp))
                Column(Modifier.padding(horizontal = 4.dp, vertical = 0.dp)) {
                    Row {
                        Text(
                            amr.name,
                            style = MaterialTheme.typography.titleMedium,
                            fontWeight = FontWeight.Bold
                        )
                        Spacer(Modifier.weight(1f))
                        Text("${amr.battery}%", color = statusColor, fontWeight = FontWeight.Bold)
                    }
                    Row {
                        Text(
                            amr.status.display,
                            style = MaterialTheme.typography.bodyMedium,
                            color = statusColor
                        )
                        Spacer(Modifier.weight(1f))
                    }
                }
            }
            Spacer(Modifier.height(8.dp))
            Row {
                Icon(
                    modifier = Modifier.size(20.dp),
                    imageVector = ImageVector.vectorResource(id = R.drawable.current_location),
                    contentDescription = "현재 위치",
                    tint = Color(0xFFD1D5DC)
                )
                Spacer(Modifier.width(8.dp))
                Text("위치: ${amr.location}", style = MaterialTheme.typography.bodyMedium)

            }
            Row {
                Icon(
                    modifier = Modifier.size(20.dp),
                    imageVector = ImageVector.vectorResource(id = R.drawable.fast_forward),
                    contentDescription = "속도",
                    tint = Color(0xFFD1D5DC)
                )
                Spacer(Modifier.width(8.dp))
                Text("속도: ${amr.speed} m/s", style = MaterialTheme.typography.bodyMedium)
            }
            Row {
                Icon(
                    modifier = Modifier.size(20.dp),
                    imageVector = ImageVector.vectorResource(id = R.drawable.check_box),
                    contentDescription = "작업",
                    tint = Color(0xFFD1D5DC)
                )
                Spacer(Modifier.width(8.dp))
                Text("작업: ${amr.job}", style = MaterialTheme.typography.bodyMedium)

            }

            Spacer(Modifier.height(16.dp))

            HorizontalDivider(thickness = 1.dp, color = Color(0xFFF5F6F8))

            Spacer(Modifier.height(16.dp))
            Row(
                verticalAlignment = Alignment.CenterVertically
            ) {
                Icon(
                    modifier = Modifier
                        .size(20.dp)
                        .rotate(90f),
                    imageVector = ImageVector.vectorResource(id = R.drawable.battery),
                    contentDescription = "배터리",
                    tint = Color(0xFFD1D5DC),
                )
                Spacer(Modifier.width(8.dp))
                LinearProgressIndicator(
                    progress = amr.battery.toFloat() / 100f,
                    modifier = Modifier
                        .fillMaxWidth()
                        .height(6.dp)
                        .background(Color(0xFFE5E5E5), shape = RoundedCornerShape(3.dp)),
                    color = statusColor
                )
            }

        }
    }
}


@Preview(showBackground = true)
@Composable
fun AmrCategoryTabRowPreview() {
    SSAMRTheme {
        AmrCategoryTabRow(
            AmrState()
        ) {}
    }

}

@Preview(showBackground = true)
@Composable
fun AmrCardListPreview() {
    val sampleAmrs = listOf(
        AmrUiModel(
            id = 1L,
            name = "AMR-001",
            status = AmrStatus.RUNNING,
            location = "A구역-라인1",
            speed = "1.2",
            job = "화물 운반 중",
            battery = 85,
        ),
        AmrUiModel(
            id = 2L,
            name = "AMR-002",
            status = AmrStatus.CHARGING,
            location = "충전소-1번",
            speed = "0",
            job = "충전 중",
            battery = 45,
        ),
        AmrUiModel(
            id = 3L,
            name = "AMR-003",
            status = AmrStatus.CHECK,
            location = "B구역-라인3",
            speed = "0",
            job = "점검 중",
            battery = 92,
        )
    )
    SSAMRTheme {
        Column(Modifier.padding(16.dp)) {
            AmrCardList(amrs = sampleAmrs) { }
        }
    }
}