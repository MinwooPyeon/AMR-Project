package com.android.ssamr.feature.login

import androidx.compose.runtime.Composable
import androidx.compose.runtime.LaunchedEffect
import androidx.hilt.navigation.compose.hiltViewModel
import androidx.navigation.NavController
import com.android.ssamr.main.navigation.navigateAndClearBackStack

@Composable
fun LoginRoute(
    navController: NavController,
    viewModel: LoginViewModel = hiltViewModel()
) {
    val state = viewModel.uiState.value

    LaunchedEffect(state.isSuccess) {
        if (state.isSuccess) {
            navController.navigateAndClearBackStack("main")
        }
    }

    LoginScreen(
        state = state,
        onIntent = { viewModel.onEvent(it) }
    )
}