package com.android.ssamr.feature.amrDetail

import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.material3.CircularProgressIndicator
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.tooling.preview.Preview
import androidx.compose.ui.unit.dp
import com.android.ssamr.R
import com.android.ssamr.core.domain.model.AmrAction
import com.android.ssamr.core.domain.model.AmrDetailAction
import com.android.ssamr.core.domain.model.AmrDetailStatus
import com.android.ssamr.core.ui.SSAMRDialog
import com.android.ssamr.ui.theme.SSAMRTheme

@Composable
fun AmrManageDetailScreen(
    state: AmrDetailState,
    sendIntent: (AmrDetailIntent) -> Unit
) {
    Box(Modifier.fillMaxSize()) {
        when {
            state.isLoading -> {
                CircularProgressIndicator(Modifier.align(Alignment.Center))
            }

            state.error != null -> {
                Text(state.error, Modifier.align(Alignment.Center))
            }

            state.showReturnDialog -> {
                SSAMRDialog(
                    iconRes = R.drawable.ic_home,
                    iconColor = Color(0xFF2563EB),
                    iconBgColor = Color(0xFFDBEAFE),
                    title = "복귀 명령 전송",
                    message = "AMR에 명령을 전송하고 있습니다...",
                    isLoading = true,
                    buttons = emptyList()
                )
            }

            state.showStartDialog -> {
                SSAMRDialog(
                    iconRes = R.drawable.ic_play,
                    iconColor = Color(0xFF2563EB),
                    iconBgColor = Color(0xFFDBEAFE),
                    title = "출발 명령 전송",
                    message = "AMR에 명령을 전송하고 있습니다...",
                    isLoading = true,
                    buttons = emptyList()
                )
            }

            state.amr != null -> {
                Column(
                    Modifier
                        .fillMaxSize()
                        .padding(16.dp)
                ) {
                    AmrDetailInfoCard(amr = state.amr)
                    Spacer(Modifier.height(32.dp))
                    AmrDetailButtonGroup(
                        onWebcamClick = { sendIntent(AmrDetailIntent.ClickWebcam(state.amr.ipAddress)) },
                        onManualReturnClick = { sendIntent(AmrDetailIntent.ClickManualReturn) },
                        onManualStartClick = { sendIntent(AmrDetailIntent.ClickManualStart) }
                    )
                }
            }
        }
    }
}

@Preview(showBackground = true)
@Composable
fun AmrManageDetailScreenPreview() {
    SSAMRTheme {
        AmrManageDetailScreen(
            state = AmrDetailState(
                amrId = 1,
                amr = sampleAmrDetail,
            )
        ) { }
    }
}

val sampleAmrDetail =
    AmrDetailStatus(
        name = "AMR-001",
        status = AmrDetailAction.RUNNING,
        location = "A구역-라인1",
        speed = "1.2m/s",
        job = "화물 운반 중",
        model = "RB-100",
        serial = "RB100-2024-001",
        firmware = "v2.1.3",
        ipAddress = "111.111.1111",
    )