package com.android.ssamr.core.data.model.amr.response

import com.android.ssamr.core.domain.model.AmrDetailAction
import com.android.ssamr.core.domain.model.AmrDetailStatus

data class AmrDetailDto(
    val id: Long,
    val name: String,
    val state: String,
    val locationX: Double,
    val locationY: Double,
    val speed: Double,
    val model: String,
    val serial: String,
    val firmware: String,
    val ipAddress: String
)

fun AmrDetailDto.toDetailModel(): AmrDetailStatus = AmrDetailStatus(
    id = this.id,
    name = this.name,
    status = AmrDetailAction.valueOf(state),
    locationX = this.locationX,
    locationY = this.locationY,
    speed = this.speed.toString(),
    job = this.state,
    model = this.model,
    serial = this.serial,
    firmware = this.firmware,
    ipAddress = this.ipAddress
)