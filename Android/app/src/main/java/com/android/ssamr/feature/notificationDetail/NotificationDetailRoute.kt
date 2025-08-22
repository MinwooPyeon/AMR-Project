package com.android.ssamr.feature.notificationDetail

import android.widget.Toast
import androidx.compose.runtime.Composable
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.collectAsState
import androidx.compose.runtime.getValue
import androidx.compose.ui.platform.LocalContext
import androidx.hilt.navigation.compose.hiltViewModel

@Composable
fun NotificationDetailRoute(
    navigateToPhotoView: (String) -> Unit,
    onBack: () -> Unit,
    viewModel: NotificationDetailViewModel = hiltViewModel()
) {
    val context = LocalContext.current
    val state by viewModel.state.collectAsState()

    // 효과(네비/토스트)만 여기서 처리
    LaunchedEffect(Unit) {
        viewModel.effect.collect { effect ->
            when (effect) {
                is NotificationDetailEffect.NavigateToPhotoView ->
                    effect.url?.let(navigateToPhotoView)

                is NotificationDetailEffect.ShowError ->
                    Toast.makeText(context, "오류가 발생했습니다. ${effect.message}", Toast.LENGTH_SHORT).show()
            }
        }
    }

    // ✅ 실 UI는 LaunchedEffect 밖에서 호출
    NotificationDetailScreen(
        state = state,
        onBack = onBack,
        sendIntent = viewModel::sendIntent
    )
}