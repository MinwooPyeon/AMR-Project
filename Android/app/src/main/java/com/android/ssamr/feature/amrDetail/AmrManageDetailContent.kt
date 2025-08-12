package com.android.ssamr.feature.amrDetail

import androidx.compose.foundation.background
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.PaddingValues
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
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Surface
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.res.painterResource
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.tooling.preview.Preview
import androidx.compose.ui.unit.dp
import com.android.ssamr.R
import com.android.ssamr.core.domain.model.AmrDetailStatus
import com.android.ssamr.ui.theme.SSAMRTheme

@Composable
fun AmrDetailInfoCard(
    amr: AmrDetailStatus,
    modifier: Modifier = Modifier
) {
//    val amrStatus = AmrDetailAction.fromStatus(amr.status)
    val amrStatus = amr.state
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
                        amr.state.display,
                        style = MaterialTheme.typography.bodyLarge,
                        color = statusColor,
                    )
                }
            }
            Spacer(Modifier.height(40.dp))

            InfoRow("현재 위치", "${amr.locationX} ${amr.locationY}")
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
        Text(title, color = Color(0xFF6D6D6D), style = MaterialTheme.typography.bodyLarge)
        Text(value, color = Color(0xFF323232), style = MaterialTheme.typography.bodyLarge)
    }
}

@Composable
fun AmrDetailBtnGroup(
    onWebcamClick: () -> Unit,
    onManualWorksheetClick: () -> Unit,
    onManualChargeClick: () -> Unit,
) {
    Column(
        verticalArrangement = Arrangement.spacedBy(16.dp),
    ) {
        Button(
            onClick = onWebcamClick,
            modifier = Modifier
                .fillMaxWidth()
                .height(56.dp),
            shape = RoundedCornerShape(16.dp),
            colors = ButtonDefaults.buttonColors(containerColor = Color(0xFF2563EB)),
            contentPadding = PaddingValues(horizontal = 20.dp)
        ) {
            Icon(
                painter = painterResource(R.drawable.ic_webcam),
                contentDescription = "웹캠",
                modifier = Modifier.size(24.dp)
            )
            Spacer(Modifier.width(8.dp))
            Text("실시간 웹캠 보기", color = Color.White, style = MaterialTheme.typography.bodyLarge)
        }
        Button(
            onClick = onManualWorksheetClick,
            modifier = Modifier
                .fillMaxWidth()
                .height(56.dp),
            shape = RoundedCornerShape(16.dp),
            colors = ButtonDefaults.buttonColors(containerColor = Color(0xFFEA580C))
        ) {
            Icon(
                painter = painterResource(R.drawable.current_location),
                contentDescription = "작업지로 이동",
                modifier = Modifier.size(24.dp)
            )
            Spacer(Modifier.width(8.dp))
            Text("작업지로 이동", color = Color.White, style = MaterialTheme.typography.bodyLarge)
        }
        Button(
            onClick = onManualChargeClick,
            modifier = Modifier
                .fillMaxWidth()
                .height(56.dp),
            shape = RoundedCornerShape(16.dp),
            colors = ButtonDefaults.buttonColors(containerColor = Color(0xFF16A34A))
        ) {
            Icon(
                painter = painterResource(R.drawable.ic_battery_charge),
                contentDescription = "충전소로 이동",
                modifier = Modifier.size(24.dp)
            )
            Spacer(Modifier.width(8.dp))
            Text("충전소로 이동", color = Color.White, style = MaterialTheme.typography.bodyLarge)
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
fun AmrDetailButtonGroupWithSheetPreview() {
    SSAMRTheme {
        AmrDetailBtnGroup(
            {}, {}
        ) {}
    }
}