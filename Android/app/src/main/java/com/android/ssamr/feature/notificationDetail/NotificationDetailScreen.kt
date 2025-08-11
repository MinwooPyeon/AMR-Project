package com.android.ssamr.feature.notificationDetail

import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.height
import androidx.compose.material3.CircularProgressIndicator
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.tooling.preview.Preview
import androidx.compose.ui.unit.dp
import com.android.ssamr.core.domain.model.Notification
import com.android.ssamr.core.domain.model.NotificationAction
import com.android.ssamr.ui.theme.SSAMRTheme

@Composable
fun NotificationDetailScreen(
    state: NotificationDetailState,
    onBack: () -> Unit,
    sendIntent: (NotificationDetailIntent) -> Unit
) {
    val n = state.data

    when {
        n != null -> {
            Column(modifier = Modifier.fillMaxSize()) {
                NotificationDetailInfoCard(notification = n)
                Spacer(Modifier.height(8.dp))
                CurrentSituationPhoto(
                    imageUrl = n.image,
                    timeText = n.date,
                    onClick = { /* 전체보기 등 */ }
                )
            }
        }
        state.isLoading -> {
            // ✅ 로딩 뷰
            Box(Modifier.fillMaxSize(), contentAlignment = Alignment.Center) {
                CircularProgressIndicator()
            }
        }
        state.error != null -> {
            // ✅ 에러 뷰
            Box(Modifier.fillMaxSize(), contentAlignment = Alignment.Center) {
                Text("불러오기 실패: ${state.error}")
            }
        }
        else -> {
            // ✅ 데이터 없음(빈 상태) – 안전망
            Box(Modifier.fillMaxSize(), contentAlignment = Alignment.Center) {
                Text("알림 정보를 찾을 수 없습니다.")
            }
        }
    }
}


@Preview(showBackground = true)
@Composable
fun NotificationDetailScreenPreview() {
    SSAMRTheme {
        NotificationDetailScreen(
            state = NotificationDetailState(
                data = sampleNotification,
                isLoading = false,
                error = null
            ),
            {}
        ){}

    }
}

val sampleNotification = Notification(
    id = 1L,
    title = "알림 제목 1",
    content = "알림 내용 1",
    riskLevel = NotificationAction.DANGER,
    location = "A구역-1",
    date = "3분전",
    image = "",)