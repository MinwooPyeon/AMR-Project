package com.android.ssamr.feature.amrDetail

import com.android.ssamr.core.ui.LocationSelectBottomSheet
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.material3.CircularProgressIndicator
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.tooling.preview.Preview
import androidx.compose.ui.unit.dp
import com.android.ssamr.R
import com.android.ssamr.core.domain.model.AmrDetailAction
import com.android.ssamr.core.domain.model.AmrDetailStatus
import com.android.ssamr.core.ui.SSAMRDialog
import com.android.ssamr.ui.theme.SSAMRTheme

@Composable
fun AmrManageDetailScreen(
    state: AmrDetailState,
    sendIntent: (AmrDetailIntent) -> Unit
) {
    var showWorksheetSheet by remember { mutableStateOf(false) }
    var showChargeSheet by remember { mutableStateOf(false) }

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
                    AmrDetailBtnGroup(
                        onWebcamClick = { sendIntent(AmrDetailIntent.ClickWebcam(state.amr.ipAddress)) },
                        onManualWorksheetClick = { showWorksheetSheet = true },
                        onManualChargeClick = { showChargeSheet = true }
                    )
                }
            }
        }

        if (showWorksheetSheet) {
            LocationSelectBottomSheet(
                title = "작업지 선택",
                locationList = worksheetList,
                onSelect = {
                    showWorksheetSheet = false
                    sendIntent(AmrDetailIntent.SelectedWorksheet(it))
                },
                onDismiss = { showWorksheetSheet = false }
            )
        }

        if (showChargeSheet) {
            LocationSelectBottomSheet(
                title = "충전소 선택",
                locationList = chargeList,
                onSelect = {
                    showChargeSheet = false
                    sendIntent(AmrDetailIntent.SelectedChargeStation(it))
                },
                onDismiss = { showChargeSheet = false }
            )
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
        id = 1,
        name = "AMR-001",
        state = AmrDetailAction.RUNNING,
        locationX = 0.0,
        locationY = 1.0,
        speed = "1.2m/s",
        job = "화물 운반 중",
        model = "RB-100",
        serial = "RB100-2024-001",
        firmware = "v2.1.3",
        ipAddress = "111.111.1111",
    )