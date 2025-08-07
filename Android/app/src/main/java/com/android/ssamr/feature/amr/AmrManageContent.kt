package com.android.ssamr.feature.amr

import androidx.compose.foundation.background
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.IntrinsicSize
import androidx.compose.foundation.layout.PaddingValues
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxHeight
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
import androidx.compose.material3.Icon
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Text
import androidx.compose.material3.TextButton
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.vector.ImageVector
import androidx.compose.ui.res.vectorResource
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.style.TextOverflow
import androidx.compose.ui.tooling.preview.Preview
import androidx.compose.ui.unit.dp
import com.android.ssamr.R
import com.android.ssamr.core.domain.model.AmrCategory
import com.android.ssamr.core.domain.model.AmrAction
import com.android.ssamr.core.domain.model.AmrStatus
import com.android.ssamr.ui.theme.SSAMRTheme

@Composable
fun AmrCardList(
    amrs: List<AmrStatus>,
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
    amr: AmrStatus,
    onAmrCardClick: (Long) -> Unit
) {
    val statusColor = when (amr.state) {
        AmrAction.RUNNING -> Color(0xFF4CAF50)
        AmrAction.CHARGING -> Color(0xFFF7B500)
        AmrAction.CHECKING -> Color(0xFFF7575C)
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
        Row(
            modifier = Modifier
                .fillMaxWidth()
                .padding(end = 0.dp)
                .height(IntrinsicSize.Min)
        ) {
            Column(
                modifier = Modifier
                    .padding(16.dp)
                    .weight(1f)) {
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
//                        Text("${amr.battery}%", color = statusColor, fontWeight = FontWeight.Bold)
                        }
                        Row {
                            Text(
                                amr.state.display,
                                style = MaterialTheme.typography.bodyMedium,
                                color = statusColor
                            )
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
                    Text("위치: ${amr.locationX} ${amr.locationY}", style = MaterialTheme.typography.bodyMedium)

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
            }
            Box(
                modifier = Modifier
                    .width(40.dp)
                    .fillMaxHeight()
                    .background(statusColor)
                    .clip(RoundedCornerShape(topEnd = 8.dp, bottomEnd = 8.dp)),
                contentAlignment = Alignment.Center
            ) {
                Icon(
                    modifier = Modifier
                        .size(40.dp),
                    imageVector = ImageVector.vectorResource(id = R.drawable.ic_right_arrow),
                    contentDescription = "화살표",
                    tint = Color.White
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
        AmrStatus(
            id = 1L,
            name = "AMR-001",
            state = AmrAction.RUNNING,
            locationX = 0.0,
            locationY = 1.0,
            speed = "1.2",
            job = "화물 운반 중",
        ),
        AmrStatus(
            id = 2L,
            name = "AMR-002",
            state = AmrAction.CHARGING,
            locationX = 2.0,
            locationY = 3.0,
            speed = "0",
            job = "충전 중",
        ),
        AmrStatus(
            id = 3L,
            name = "AMR-003",
            state = AmrAction.CHECKING,
            locationX = 4.0,
            locationY = 5.0,
            speed = "0",
            job = "점검 중",
        )
    )
    SSAMRTheme {
        Column(Modifier.padding(16.dp)) {
            AmrCardList(amrs = sampleAmrs) { }
        }
    }
}