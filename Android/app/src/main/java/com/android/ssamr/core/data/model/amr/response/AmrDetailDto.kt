package com.android.ssamr.core.data.model.amr.response

import com.android.ssamr.core.domain.model.AmrDetailStatus

data class AmrDetailDto(
    val name: String,
    val status: String,
    val battery: Int,
    val location: String,
    val speed: String,
    val job: String,
    val model: String,
    val serial: String,
    val firmware: String,
)

fun AmrDetailDto.toDetailModel(): AmrDetailStatus = AmrDetailStatus(
    name = this.name,
    status = this.status,
    battery = this.battery,
//    location = this.location,
    location = "A구역-1",
    speed = this.speed,
    job = this.job,
    model = this.model,
    serial = this.serial,
    firmware = this.firmware
)