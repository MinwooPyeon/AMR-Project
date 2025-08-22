package com.android.ssamr.core.ui

import androidx.compose.runtime.Composable

data class TopBarConfig(
    val title: String,
    val subTitle: String? = null,
    val showBack: Boolean = false,
    val onBackClick: (() -> Unit)? = null,
    val actions: (@Composable () -> Unit)? = null,
    val isCustom: Boolean = false // 커스텀바(예: 대시보드) 필요시
)
