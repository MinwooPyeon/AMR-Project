package com.android.ssamr.core.data.model.amr.response

import com.android.ssamr.core.domain.model.AmrMapPosition
import com.android.ssamr.core.domain.model.DashboardAmrStatus

data class DashMapDto(
    val id: Long,
    val serial: String,
    val name: String,
    val x: Float,
    val y: Float,
    val state: String
)

fun DashMapDto.toAmrMapPositionModel():  AmrMapPosition = AmrMapPosition(
    id = this.id,
    serial = this.serial,
    this.name,
    x = this.x,
    y = this.y,
    status = DashboardAmrStatus.from(state)
)