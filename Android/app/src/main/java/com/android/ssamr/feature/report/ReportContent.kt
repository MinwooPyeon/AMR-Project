package com.android.ssamr.feature.report

import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.Card
import androidx.compose.material3.CardDefaults
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.tooling.preview.Preview
import androidx.compose.ui.unit.dp
import com.android.ssamr.ui.theme.SSAMRTheme

@Composable
fun ReportStatCard(
    count: Int,
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



@Preview(showBackground = true)
@Composable
fun ReportCardPreview() {
    SSAMRTheme {
        ReportStatCard(
            count = 10,
            label = "위험",
            countColor = Color.Red
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