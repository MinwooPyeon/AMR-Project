package com.android.ssamr.feature.dashboard.fullscreenmap

import androidx.compose.runtime.Composable

@Composable
fun FullscreenMapRoute(
    navigateToAmrDetail: (Long) -> Unit,
    onBack: () -> Unit
) {
    FullscreenMapScreen(
        navigateToAmrDetail = navigateToAmrDetail,
        onBack = onBack
    )
}