package com.android.ssamr.core.data.model.amr.response

import com.android.ssamr.feature.dashboard.DashboardAmrStatus
import com.android.ssamr.feature.dashboard.fullscreenmap.AmrMapPositionModel

data class DashMapDto(
    val id: Long,
    val name: String,
    val x: Float,
    val y: Float,
    val status: DashboardAmrStatus
)

fun DashMapDto.toAmrMapPositionModel():  AmrMapPositionModel = AmrMapPositionModel(
    id = this.id,
    name = this.name,
    x = this.x,
    y = this.y,
    status = this.status
)