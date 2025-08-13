package com.android.ssamr.feature.report

import android.widget.Toast
import androidx.compose.runtime.Composable
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.collectAsState
import androidx.compose.runtime.getValue
import androidx.compose.ui.platform.LocalContext
import androidx.hilt.navigation.compose.hiltViewModel

@Composable
fun ReportRoute(
    navigateToReportDetail: (Long) -> Unit,
    onBack: () -> Unit,
    viewModel: ReportViewModel = hiltViewModel()
) {
    val context = LocalContext.current
    val state by viewModel.state.collectAsState()

    LaunchedEffect(Unit) {
        viewModel.effect.collect { effect ->
            when (effect) {

                is ReportEffect.NavigateToReportDetail -> {
                    navigateToReportDetail(effect.reportId)
                }

                is ReportEffect.ShowError -> {
                    Toast.makeText(
                        context,
                        "오류가 발생했습니다.",
                        Toast.LENGTH_SHORT
                    ).show()
                }
            }
        }
    }
    ReportScreen(
        state = state,
        sendIntent = viewModel::sendIntent
    )
}