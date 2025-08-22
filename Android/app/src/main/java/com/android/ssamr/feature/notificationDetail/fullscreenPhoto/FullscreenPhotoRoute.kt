package com.android.ssamr.feature.notificationDetail.fullscreenPhoto

import androidx.compose.runtime.Composable

@Composable
fun FullscreenPhotoRoute(
    imageUrl: String,
    onBack: () -> Unit
) {
    FullscreenPhotoScreen(imageUrl = imageUrl, onBack = onBack)
}