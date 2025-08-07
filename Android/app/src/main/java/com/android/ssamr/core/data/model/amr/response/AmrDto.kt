package com.android.ssamr.core.data.model.amr.response

import com.android.ssamr.core.domain.model.AmrAction
import com.android.ssamr.core.domain.model.AmrDetailAction
import com.android.ssamr.core.domain.model.AmrStatus

data class AmrDto(
    val id: Long,
    val name: String,
    val status: String,
    val locationX: Double,
    val locationY: Double,
    val location: String,
    val speed: String,
)

fun AmrDto.toUiModel(): AmrStatus = AmrStatus(
    id = id,
    name = name,
    status = AmrAction.valueOf(status),
    locationX = this.locationX,
    locationY = this.locationY,
    speed = speed,
    job = status,
)