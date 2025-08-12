package com.android.ssamr.core.data.model.amr.response

import com.android.ssamr.core.domain.model.AmrAction
import com.android.ssamr.core.domain.model.AmrStatus

data class AmrDto(
    val id: Long,
    val name: String,
    val state: String,
    val x: Double,
    val y: Double,
    val location: String,
    val speed: String,
    val serial: String
)

fun AmrDto.toUiModel(): AmrStatus = AmrStatus(
    id = id,
    name = name,
    state = AmrAction.valueOf(state),
    locationX = this.x,
    locationY = this.y,
    speed = speed,
    job = state,
    serial = this.serial
)