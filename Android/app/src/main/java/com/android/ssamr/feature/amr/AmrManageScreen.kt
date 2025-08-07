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
import com.android.ssamr.core.domain.model.AmrCategory
import com.android.ssamr.core.domain.model.AmrAction
import com.android.ssamr.core.domain.model.AmrDetailAction
import com.android.ssamr.core.domain.model.AmrStatus

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
                amrs = state.amrList,
//                amrs = sampleAmrs,
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
            amrList = sampleAmrs,
            categoryCounts = mapOf(
                AmrCategory.ALL to 6,
                AmrCategory.RUNNING to 3,
                AmrCategory.CHARGING to 1,
                AmrCategory.CHECKING to 2
            ),
            isLoading = false,
            error = null
        ),
        sendIntent = {}
    )
}

// 임시 데이터
val sampleAmrs = listOf(
    AmrStatus(
        id = 1L,
        name = "AMR-001",
        status = AmrAction.RUNNING,
        locationX = 0.0,
        locationY = 1.0,
        speed = "1.2",
        job = "화물 운반 중",
    ),
    AmrStatus(
        id = 2L,
        name = "AMR-002",
        status = AmrAction.CHARGING,
        locationX = 2.0,
        locationY = 3.0,
        speed = "1.2",
        job = "충전 중",
    ),
    AmrStatus(
        id = 3L,
        name = "AMR-003",
        status = AmrAction.CHECKING,
        locationX = 4.0,
        locationY = 5.0,
        speed = "1.2",
        job = "점검 중",
    )
)