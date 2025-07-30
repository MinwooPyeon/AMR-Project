package com.android.ssamr.feature.amr

import android.widget.Toast
import androidx.compose.runtime.Composable
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.collectAsState
import androidx.compose.runtime.getValue
import androidx.compose.ui.platform.LocalContext
import androidx.hilt.navigation.compose.hiltViewModel

@Composable
fun AmrManageRoute(
    navigateToAmrDetail: (Long) -> Unit,
    viewModel: AmrManageViewModel = hiltViewModel(),
    onRefresh: ((() -> Unit) -> Unit)? = null
) {
    val context = LocalContext.current
    val state by viewModel.state.collectAsState()

    LaunchedEffect(Unit) {
        viewModel.sendIntent(AmrIntent.ClickAmrCategory(AmrCategory.ALL))
    }

    LaunchedEffect(Unit) {
        viewModel.effect.collect { effect ->
            when (effect) {

                is AmrEffect.NavigateToAmrDetail -> {
                    navigateToAmrDetail(effect.amrId)
                }

                is AmrEffect.ShowError -> {
                    Toast.makeText(
                        context,
                        "오류가 발생했습니다.",
                        Toast.LENGTH_SHORT
                    ).show()
                }
            }
        }
    }

    onRefresh?.invoke { viewModel.refreshAmrList() }

    AmrManageScreen(
        state = state,
        sendIntent = viewModel::sendIntent
    )
}