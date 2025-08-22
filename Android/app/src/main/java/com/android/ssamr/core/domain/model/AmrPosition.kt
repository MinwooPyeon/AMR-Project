package com.android.ssamr.core.domain.model

data class AmrMapPosition(
    val id: Long,
    val serial: String,
    val name: String,
    val x: Float, // px 또는 meter 기준: 추후 확정 필요
    val y: Float,
    val status: DashboardAmrStatus
)