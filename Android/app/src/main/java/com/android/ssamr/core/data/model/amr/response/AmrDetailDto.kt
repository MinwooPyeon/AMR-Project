package com.android.ssamr.core.data.model.amr.response

import com.android.ssamr.core.domain.model.AmrDetailAction
import com.android.ssamr.core.domain.model.AmrDetailStatus

data class AmrDetailDto(
    val name: String,
    val status: String,
    val battery: Int,
    val speed: Double,
    val model: String,
    val serial: String,
    val firmware: String,
)

fun AmrDetailDto.toDetailModel(): AmrDetailStatus = AmrDetailStatus(
    name = this.name,
    status = AmrDetailAction.valueOf(status),
//    status = status,
    battery = this.battery,
//    location = this.location,
    location = "A구역-1",
    speed = this.speed.toString(),
    job = this.status,
    model = this.model,
    serial = this.serial,
    firmware = this.firmware
)