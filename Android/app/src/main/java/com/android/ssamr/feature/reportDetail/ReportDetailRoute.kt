package com.android.ssamr.feature.reportDetail

import android.widget.Toast
import androidx.compose.runtime.Composable
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.collectAsState
import androidx.compose.runtime.getValue
import androidx.compose.ui.platform.LocalContext
import androidx.hilt.navigation.compose.hiltViewModel
import com.android.ssamr.feature.report.ReportEffect
import com.android.ssamr.main.navigation.ReportDetailScreen

@Composable
fun ReportDetailRoute(
    id: Long,
    onBack: () -> Unit,
    viewModel: ReportDetailViewModel = hiltViewModel()
) {
    val context = LocalContext.current
    val state by viewModel.state.collectAsState()

    LaunchedEffect(Unit) {
        viewModel.effect.collect { effect ->
            when (effect) {

                is ReportDetailEffect.ShowError -> {
                    Toast.makeText(
                        context,
                        "이벤트 상세에서 오류가 발생했습니다. ${effect.message}",
                        Toast.LENGTH_SHORT
                    ).show()
                }
            }
        }
    }

    ReportDetailScreen(
        state = state,
        sendIntent = viewModel::sendIntent,
    )
}