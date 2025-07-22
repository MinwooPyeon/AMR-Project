package com.android.ssamr.feature.amr

import androidx.compose.foundation.background
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.material3.CircularProgressIndicator
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.tooling.preview.Preview
import androidx.compose.ui.unit.dp
import com.android.ssamr.ui.theme.SSAMRTheme

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