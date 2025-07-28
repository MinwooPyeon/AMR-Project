package com.android.ssamr.main.navigation

import androidx.navigation.NavController
import androidx.navigation.NavOptionsBuilder

fun NavController.navigateAndClearBackStack(
    route: String,
    builder: NavOptionsBuilder.() -> Unit = {}
) {
    navigate(route) {
        popUpTo(graph.startDestinationId) {
            inclusive = true
        }
        builder()
    }
}