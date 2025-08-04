package com.example.amr_backend.v1.dto

import com.example.amr_backend.v1.entity.AmrStatus
import com.example.amr_backend.v1.entity.Status

data class AmrStatusResponse(
    val id: Long,
    val name: String,
    val status: Status,
    val speed: Double,
    val battery: Int
)

fun AmrStatus.toAmrStatusResponse() = AmrStatusResponse(
    id = id,
    name = amr.name,
    status = status,
    speed = speed,
    battery = batteryLevel
)