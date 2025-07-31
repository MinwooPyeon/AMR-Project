package com.android.ssamr.core.data.model.amr.response

import com.android.ssamr.feature.amr.AmrStatus
import com.android.ssamr.feature.amr.AmrUiModel

data class AmrDto(
    val id: Long,
    val name: String,
    val status: String,
    val location: String,
    val speed: String,
    val battery: Int
)

fun AmrDto.toUiModel(): AmrUiModel = AmrUiModel(
    id = this.id,
    name = this.name,
    status = AmrStatus.valueOf(this.status),
    location = "A구역-1",
    speed = this.speed,
    job = this.status,
    battery = this.battery
)