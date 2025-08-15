package com.android.ssamr.feature.notification

import android.content.Intent
import android.net.Uri
import android.util.Log
import android.widget.Toast
import androidx.compose.runtime.Composable
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.collectAsState
import androidx.compose.runtime.getValue
import androidx.compose.ui.platform.LocalContext
import androidx.hilt.navigation.compose.hiltViewModel
import com.android.ssamr.core.domain.model.NotificationCategory

@Composable
fun NotificationRoute(
    navigateToNotificationDetail: (Long) -> Unit,
    viewModel: NotificationViewModel = hiltViewModel(),
    onRefresh: ((() -> Unit) -> Unit)? = null,
    onCallClick: ((() -> Unit) -> Unit)? = null
) {
    val context = LocalContext.current
    val state by viewModel.state.collectAsState()

    LaunchedEffect(Unit) {
        viewModel.sendIntent(NotificationIntent.ClickNotificationCategory(NotificationCategory.ALL))
    }

    LaunchedEffect(Unit) {
        viewModel.effect.collect { effect ->
            when (effect) {
                is NotificationEffect.NavigateToNotificationDetail -> {
                    navigateToNotificationDetail(effect.notificationId)
                }

                is NotificationEffect.ShowError -> {
                    Toast.makeText(
                        context,
                        "알림창에 오류가 발생했습니다. ${state.error}",
                        Toast.LENGTH_SHORT
                    ).show()
                    Log.d("TAG", "NotificationRoute: ${state.error}")
                }

                is NotificationEffect.LaunchDialer -> {
                    val intent = Intent(Intent.ACTION_DIAL, Uri.parse("tel:${effect.phone}"))
                    context.startActivity(intent)
                }
            }
        }
    }

    LaunchedEffect(onRefresh)   { onRefresh?.invoke { viewModel.syncFromServer() } }
    LaunchedEffect(onCallClick) { onCallClick?.invoke { viewModel.openCallDialog() } }

    NotificationScreen(
        state = state,
        sendIntent = viewModel::sendIntent
    )
}