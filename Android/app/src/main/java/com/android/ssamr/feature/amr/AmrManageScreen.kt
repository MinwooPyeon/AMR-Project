package com.android.ssamr.feature.amr

import androidx.compose.foundation.background
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.height
import androidx.compose.runtime.Composable
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.tooling.preview.Preview
import androidx.compose.ui.unit.dp

@Composable
fun AmrManageScreen(
    state: AmrState = AmrState(),
    sendIntent: (AmrIntent) -> Unit = {}
) {
    Box(
        modifier = Modifier
            .fillMaxSize()
            .background(Color(0xFFF4F6FA))
    ) {
        Column(modifier = Modifier.fillMaxSize()) {
            // 카테고리 탭
            AmrCategoryTabRow(state = state, sendIntent = sendIntent)

            Spacer(Modifier.height(8.dp))

            // AMR 카드 리스트
            AmrCardList(
//                amrs = state.amrList,
                amrs = sampleAmrs,
                onAmrCardClick = { amrId ->
                    sendIntent(AmrIntent.ClickAmrManageCard(amrId))
                }
            )
        }
    }
}

@Preview(showBackground = true)
@Composable
fun AmrManageScreenPreview() {
    AmrManageScreen(
        state = AmrState(
            selectedCategory = AmrCategory.RUNNING,
            amrList = listOf(
                // 샘플 AmrUiModel들
            ),
            categoryCounts = mapOf(
                AmrCategory.ALL to 6,
                AmrCategory.RUNNING to 3,
                AmrCategory.CHARGING to 1,
                AmrCategory.CHECK to 2
            ),
            isLoading = false,
            error = null
        ),
        sendIntent = {}
    )
}

// 임시 데이터
val sampleAmrs = listOf(
    AmrUiModel(
        id = 1L,
        name = "AMR-001",
        status = AmrStatus.RUNNING,
        location = "A구역-라인1",
        speed = "1.2",
        job = "화물 운반 중",
        battery = 85,
        lastUpdated = "2분 전"
    ),
    AmrUiModel(
        id = 2L,
        name = "AMR-002",
        status = AmrStatus.CHARGING,
        location = "충전소-1번",
        speed = "0",
        job = "충전 중",
        battery = 45,
        lastUpdated = "5분 전"
    ),
    AmrUiModel(
        id = 3L,
        name = "AMR-003",
        status = AmrStatus.CHECK,
        location = "B구역-라인3",
        speed = "0",
        job = "점검 중",
        battery = 92,
        lastUpdated = "1시간 전"
    )
)