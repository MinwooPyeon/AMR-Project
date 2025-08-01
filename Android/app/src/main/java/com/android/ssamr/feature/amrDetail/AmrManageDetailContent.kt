package com.android.ssamr.feature.amrDetail

import androidx.compose.foundation.background
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.Button
import androidx.compose.material3.ButtonDefaults
import androidx.compose.material3.Icon
import androidx.compose.material3.LinearProgressIndicator
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Surface
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.res.painterResource
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.tooling.preview.Preview
import androidx.compose.ui.unit.dp
import com.android.ssamr.R
import com.android.ssamr.core.domain.model.AmrDetailAction
import com.android.ssamr.core.domain.model.AmrDetailStatus
import com.android.ssamr.ui.theme.SSAMRTheme

@Composable
fun AmrDetailInfoCard(
    amr: AmrDetailStatus,
    modifier: Modifier = Modifier
) {
//    val amrStatus = AmrDetailAction.fromStatus(amr.status)
    val amrStatus = amr.status
    val statusColor = amrStatus.color

    Surface(
        shape = RoundedCornerShape(24.dp),
        color = Color.White,
        shadowElevation = 2.dp,
        modifier = modifier
    ) {
        Column(Modifier.padding(24.dp)) {
            Row(verticalAlignment = Alignment.CenterVertically) {
                Icon(
                    painter = painterResource(R.drawable.ic_amr),
                    contentDescription = "AMR 아이콘",
                    tint = Color.White,
                    modifier = Modifier
                        .size(56.dp)
                        .background(
                            color = statusColor,
                            shape = CircleShape
                        )
                        .padding(12.dp)
                )
                Spacer(Modifier.width(16.dp))
                Column {
                    Text(
                        amr.name,
                        style = MaterialTheme.typography.titleLarge.copy(fontWeight = FontWeight.Bold)
                    )
                    Text(
                        amr.status.display,
                        style = MaterialTheme.typography.bodyLarge,
                        color = statusColor,
                    )
                }
            }
            Spacer(Modifier.height(16.dp))

            Text(
                "배터리", color = Color(0xFFBCC0C6), style = MaterialTheme.typography.bodyMedium
            )
            Spacer(Modifier.height(6.dp))
            LinearProgressIndicator(
                progress = amr.battery / 100f,
                modifier = Modifier
                    .fillMaxWidth()
                    .height(10.dp)
                    .clip(RoundedCornerShape(6.dp)),
                color = statusColor,
                trackColor = Color(0xFFE5E5E5)
            )
            Row(
                Modifier.fillMaxWidth(),
                horizontalArrangement = Arrangement.End
            ) {
                Text("${amr.battery}%", color = Color(0xFF7C838A))
            }

            Spacer(Modifier.height(24.dp))

            InfoRow("현재 위치", amr.location)
            InfoRow("이동 속도", amr.speed)
            InfoRow("현재 작업", amr.job)
            InfoRow("모델명", amr.model)
            InfoRow("시리얼 번호", amr.serial)
            InfoRow("펌웨어 버전", amr.firmware)
        }
    }
}


@Composable
private fun InfoRow(title: String, value: String) {
    Row(
        Modifier
            .fillMaxWidth()
            .padding(vertical = 2.dp),
        horizontalArrangement = Arrangement.SpaceBetween
    ) {
        Text(title, color = Color(0xFF6D6D6D), style = MaterialTheme.typography.bodyMedium)
        Text(value, color = Color(0xFF323232), style = MaterialTheme.typography.bodyMedium)
    }
}

@Composable
fun AmrDetailButtonGroup(
    onWebcamClick: () -> Unit,
    onManualReturnClick: () -> Unit,
    onManualStartClick: () -> Unit,
    modifier: Modifier = Modifier
) {
    Column(
        modifier = modifier,
        verticalArrangement = Arrangement.spacedBy(16.dp)
    ) {
        Button(
            onClick = onWebcamClick,
            modifier = Modifier.fillMaxWidth(),
            colors = ButtonDefaults.buttonColors(containerColor = Color(0xFF2167EA))
        ) {
            Icon(
                painter = painterResource(R.drawable.ic_webcam),
                contentDescription = "웹캠",
                modifier = Modifier.size(20.dp)
            )
            Spacer(Modifier.width(8.dp))
            Text("실시간 웹캠 보기", color = Color.White)
        }

        Button(
            onClick = onManualReturnClick,
            modifier = Modifier.fillMaxWidth(),
            colors = ButtonDefaults.buttonColors(containerColor = Color(0xFFFF7024))
        ) {
            Icon(
                painter = painterResource(R.drawable.ic_home),
                contentDescription = "복귀",
                modifier = Modifier.size(20.dp)
            )
            Spacer(Modifier.width(8.dp))
            Text("수동 복귀", color = Color.White)
        }

        Button(
            onClick = onManualStartClick,
            modifier = Modifier.fillMaxWidth(),
            colors = ButtonDefaults.buttonColors(containerColor = Color(0xFF23C06C))
        ) {
            Icon(
                painter = painterResource(R.drawable.ic_play),
                contentDescription = "출발",
                modifier = Modifier.size(20.dp)
            )
            Spacer(Modifier.width(8.dp))
            Text("수동 출발", color = Color.White)
        }
    }
}

@Preview(showBackground = true)
@Composable
fun AmrDetailInfoCardPreview() {
    SSAMRTheme {
        AmrDetailInfoCard(
            amr = sampleAmrDetail
        )
    }
}

@Preview(showBackground = true)
@Composable
fun AmrDetailButtonPreview() {
    SSAMRTheme {
        AmrDetailButtonGroup(
            {}, {}, {}
        )
    }
}